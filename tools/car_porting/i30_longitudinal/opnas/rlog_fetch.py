# ruff: noqa: B905  (host analysis runs Python 3.8, which has no zip(strict=))
"""Second fetch path: raw segment log via the OP-NAS log route, parsed locally (Docker).

GET /api/log-data/drives/{device}/{route}/segments/{n}/log, checks X-Log-SHA256, parses
with openpilot LogReader, and writes the same per-segment API-format file as fetch.py
(plus source/sha256 provenance). The raw log is not kept. Runs in the repository Docker
image (needs LogReader and the zstandard package). Order: C3X segments, reverse of the
interleaved query order, so the two fetchers do not collide.
"""

import base64
import bz2
import gzip
import hashlib
import json
import os
import sys
import tempfile
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import fetch

import zstandard
from openpilot.tools.lib.logreader import LogReader


def getter(path):
  parts = path.split('.')

  def get(msg):
    x = msg
    for p in parts:
      x = getattr(x, p)
    return convert_value(x)

  return get


def convert_value(x):
  if isinstance(x, (bool, int, float, str)):
    return x
  try:
    return [convert_value(v) for v in x]
  except TypeError:
    return str(x)


def parse(path, dev, route, seg, sha):
  chans = {ch: dict(fields=['t'] + f, rows=[]) for ch, f in fetch.SIGNALS.items()}
  getters = {ch: [getter(f) for f in fs] for ch, fs in fetch.SIGNALS.items() if ch not in ('can', 'sendcan')}
  damaged = None
  try:
    for ev in LogReader(path):
      w = ev.which()
      if w not in chans:
        continue
      msg = getattr(ev, w)
      if w in ('can', 'sendcan'):
        chans[w]['rows'].append([ev.logMonoTime, [m.address for m in msg], [m.src for m in msg], [base64.b64encode(m.dat).decode() for m in msg]])
      else:
        row = [ev.logMonoTime]
        for g in getters[w]:
          try:
            v = g(msg)
            row.append(None if isinstance(v, float) and v != v else v)
          except Exception:
            row.append(None)
        chans[w]['rows'].append(row)
  except Exception as e:  # truncated log: keep what was read, like the API's "damaged"
    damaged = repr(e)
  for c in chans.values():
    c['count'] = len(c['rows'])
  if not chans['can']['rows']:
    raise RuntimeError(f'no CAN parsed ({damaged})')
  lines = [
    dict(type='start', device_id=dev, route=route, segments=[seg], source='rlog', sha256=sha),
    dict(type='segment', segment=seg, processor='local-rlog', status='ok', damaged=damaged, channels=chans),
    dict(type='end', complete=True, segments_read=1, failed=[], missing=[], source='rlog'),
  ]
  return gzip.compress('\n'.join(json.dumps(x) for x in lines).encode() + b'\n')


def one(job):
  dev, route, seg = job
  out = fetch.ROOT / 'raw' / dev / route / f'{seg}.jsonl.gz'
  if out.exists():
    return 'cached'
  out.parent.mkdir(parents=True, exist_ok=True)
  url = f"{fetch.BASE}/api/log-data/drives/{dev}/{route}/segments/{seg}/log"
  for attempt in range(3):
    try:
      with urllib.request.urlopen(url, timeout=600) as r:
        data, name, sha = r.read(), r.headers.get('X-Log-Name', ''), r.headers.get('X-Log-SHA256')
      if sha and hashlib.sha256(data).hexdigest() != sha:
        raise RuntimeError('sha256 mismatch')
      if name.endswith('.zst') or data[:4] == b'\x28\xb5\x2f\xfd':
        data = zstandard.ZstdDecompressor().decompressobj().decompress(data)
      elif name.endswith('.bz2') or data[:3] == b'BZh':
        data = bz2.decompress(data)
      with tempfile.NamedTemporaryFile(prefix='rlog', dir='/tmp') as tmp:  # LogReader rejects unknown extensions
        tmp.write(data)
        tmp.flush()
        blob = parse(tmp.name, dev, route, seg, sha)
      if out.exists():
        return 'raced'
      t = out.with_suffix(f'.{os.getpid()}.tmp')
      t.write_bytes(blob)
      t.rename(out)
      return 'ok'
    except Exception as e:
      err = repr(e)
      time.sleep(15 * (attempt + 1))
  (out.parent / f'{seg}.rlog-error.txt').write_text(err)
  return 'error ' + err


def main():
  order = json.load(open(fetch.ROOT / 'fetch-order.json'))
  c3x = [tuple(j) for j in order if j[0] == 'ff528bde139e99de'][::-1]
  t0 = time.time()
  with ThreadPoolExecutor(int(sys.argv[1]) if len(sys.argv) > 1 else 3) as ex:
    for i, (job, res) in enumerate(zip(c3x, ex.map(one, c3x))):
      if res != 'cached':
        print(i, round(time.time() - t0), *job, res, flush=True)


if __name__ == '__main__':
  main()
