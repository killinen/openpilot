# ruff: noqa: B905  (host analysis runs Python 3.8, which has no zip(strict=))
"""Convert OP-NAS per-segment API responses into compact npz arrays (host, numpy only).

CAN is decoded directly from the i30 DBC so pedal/RPM/speed semantics do not depend
on the software revision that recorded the log.
"""

import base64
import gzip
import json
import re
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
import os

REPO = os.environ.get('I30_REPO', str(Path(__file__).resolve().parents[4]))

ROOT = Path(os.environ['I30_LONG_DATA'])  # data directory outside the repository (required)
DBC = Path(REPO + '/opendbc/hyundai_i30_2014.dbc')
WANT = {
  'EMS_DCT1': ['PV_AV_CAN', 'N', 'TQI', 'TQI_ACOR', 'TQFR', 'TQ_STND'],
  'EMS_DCT2': ['BRAKE_ACT'],
  'EMS1': ['N', 'VS', 'TQI', 'TQFR'],
  'EMS2': ['TPS', 'PV_AV_CAN', 'CLU_ACK', 'BRAKE_ACT', 'ACC_ACT'],
  'EMS6': ['TQI_TARGET', 'TQI_MIN', 'TQI_MAX', 'CF_Ems_AclAct', 'CRUISE_LAMP_S'],
  'ESP2': ['LONG_ACCEL', 'LONG_ACCEL_STAT', 'LONG_ACCEL_DIAG', 'YAW_RATE', 'CYL_PRES'],
  'TCS5': ['WHEEL_FL', 'WHEEL_FR', 'WHEEL_RL', 'WHEEL_RR'],
  'GAS_SENSOR': ['INTERCEPTOR_GAS', 'INTERCEPTOR_GAS2', 'STATE'],
  'GAS_COMMAND': ['GAS_COMMAND', 'GAS_COMMAND2', 'ENABLE'],
  'CLU2': ['CF_Clu_SwiGearR'],
}


def parse_dbc():
  msgs, cur = {}, None
  for line in DBC.read_text(errors='ignore').splitlines():
    m = re.match(r'\s*BO_ (\d+) (\w+):', line)
    if m:
      cur = m.group(2) if m.group(2) in WANT else None
      if cur:
        msgs[cur] = dict(address=int(m.group(1)), signals={})
      continue
    m = re.match(r'\s*SG_ (\w+) : (\d+)\|(\d+)@([01])([+-]) \(([^,]+),([^)]+)\)', line)
    if m and cur and m.group(1) in WANT[cur]:
      msgs[cur]['signals'][m.group(1)] = (int(m.group(2)), int(m.group(3)), m.group(4) == '1', m.group(5) == '-', float(m.group(6)), float(m.group(7)))
  return msgs


MSGS = parse_dbc()


def decode(payloads, sig):
  start, length, little, signed, scale, offset = sig
  raw = np.zeros((len(payloads), 8), np.uint8)
  for i, p in enumerate(payloads):
    raw[i, : len(p)] = np.frombuffer(p, np.uint8)
  if little:
    word = raw.view('<u8')[:, 0]
    val = (word >> np.uint64(start)) & np.uint64((1 << length) - 1)
  else:
    word = raw.view('>u8')[:, 0]
    pos = (start // 8) * 8 + 7 - (start % 8)
    val = (word >> np.uint64(64 - pos - length)) & np.uint64((1 << length) - 1)
  val = val.astype(np.int64)
  if signed:
    val = np.where(val >= 1 << (length - 1), val - (1 << length), val)
  return val * scale + offset


def convert(path):
  out = path.with_suffix('').with_suffix('.npz')
  if out.exists() and out.stat().st_mtime > path.stat().st_mtime:
    return 'cached'
  arrays, meta = {}, {}
  for line in gzip.open(path):
    item = json.loads(line)
    if item.get('type') == 'segment':
      meta['damaged'] = item.get('damaged')
      for ch, c in item['channels'].items():
        fields, rows = c['fields'], c['rows']
        if ch in ('can', 'sendcan'):
          by = {}
          for r in rows:
            t = r[0]
            for a, s, d in zip(r[1], r[2], r[3]):
              by.setdefault((a, s), ([], []))
              by[(a, s)][0].append(t)
              by[(a, s)][1].append(base64.b64decode(d))
          for name, m in MSGS.items():
            for (a, s), (ts, ds) in by.items():
              if a != m['address']:
                continue
              key = f'{ch}.{name}.{s}'
              arrays[key + '.t'] = np.array(ts, np.int64)
              for sn, sig in m['signals'].items():
                arrays[f'{key}.{sn}'] = decode(ds, sig).astype(np.float32)
          continue
        if ch in ('initData', 'carParams'):
          meta[ch] = rows[0][1:] if rows else None
          continue
        if not rows:
          continue
        arrays[f'{ch}.t'] = np.array([r[0] for r in rows], np.int64)
        for j, f in enumerate(fields[1:], 1):
          col = [r[j] for r in rows]
          if any(isinstance(v, list) for v in col):
            width = max((len(v) for v in col if isinstance(v, list)), default=0)
            if width == 0:
              continue
            arr = np.full((len(col), width), np.nan, np.float32)
            for i, v in enumerate(col):
              if isinstance(v, list) and len(v) == width:
                arr[i] = [np.nan if x is None or isinstance(x, (list, str)) else x for x in v]
            arrays[f'{ch}.{f}'] = arr
          elif any(isinstance(v, str) for v in col):
            arrays[f'{ch}.{f}'] = np.array([v if isinstance(v, str) else '' for v in col])
          else:
            arrays[f'{ch}.{f}'] = np.array([np.nan if v is None else float(v) for v in col], np.float32)
    elif item.get('type') == 'end':
      meta['complete'] = item.get('complete')
  arrays['meta'] = np.array(json.dumps(meta))
  np.savez_compressed(out, **arrays)
  return 'ok'


if __name__ == '__main__':
  files = sorted((ROOT / 'raw').glob('*/*/*.jsonl.gz'))
  with ProcessPoolExecutor(int(sys.argv[1]) if len(sys.argv) > 1 else 6) as ex:
    for f, r in zip(files, ex.map(convert, files, chunksize=4)):
      if r != 'cached':
        print(f, r, flush=True)
