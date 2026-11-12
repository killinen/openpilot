# ruff: noqa: B905  (host analysis runs Python 3.8, which has no zip(strict=))
import os
from pathlib import Path
import numpy as np

ROOT = Path(os.environ['I30_LONG_DATA'])  # data directory outside the repository (required)


def load(devices=None, routes=None, cols=None):
  """Concatenate feature segments; a column missing from a segment is NaN-filled so all columns stay row-aligned."""
  parts = []
  for p in sorted((ROOT / 'features').glob('*/*/*.npz')):
    dev, route, seg = p.parent.parent.name, p.parent.name, int(p.stem)
    if devices and dev not in devices:
      continue
    if routes and route not in routes:
      continue
    d = np.load(p)
    if 'empty' in d.files:
      continue
    keys = [k for k in (cols or d.files) if k != 'meta' and (k in d.files or cols)]
    parts.append((dev, route, seg, len(d['t']), {k: d[k] for k in keys if k in d.files}))
  if not parts:
    return {}
  names = cols or sorted({k for *_, f in parts for k in f})
  out = {}
  for dev, route, seg, n, f in parts:
    for k in names:
      if k == 'meta':
        continue
      v = f[k] if k in f else np.full(n, np.nan)
      out.setdefault(k, []).append(v.astype(np.float64) if k != 't' else v)
    out.setdefault('device', []).append(np.full(n, dev, dtype=object))
    out.setdefault('route', []).append(np.full(n, route, dtype=object))
    out.setdefault('segment', []).append(np.full(n, seg))
  res = {k: np.concatenate(v) for k, v in out.items()}
  assert len({len(v) for v in res.values()}) == 1, 'misaligned columns'
  return res


def blocks(d, max_gap_s=0.05):
  """Index ranges of contiguous samples (same route/segment, no time gap)."""
  t = d['t']
  brk = np.flatnonzero((np.diff(t) > max_gap_s * 1e9) | (d['segment'][1:] != d['segment'][:-1]) | (d['route'][1:] != d['route'][:-1])) + 1
  edges = np.r_[0, brk, len(t)]
  return list(zip(edges[:-1], edges[1:]))


def deriv(d, x, half=12):
  """Centered difference over +-half samples, NaN near block edges."""
  y = np.full(len(x), np.nan)
  for a, b in blocks(d):
    if b - a > 2 * half + 1:
      y[a + half : b - half] = (x[a + 2 * half : b] - x[a : b - 2 * half]) / (2 * half * 0.02)
  return y


def smooth(d, x, half=12):
  y = np.full(len(x), np.nan)
  k = np.ones(2 * half + 1) / (2 * half + 1)
  for a, b in blocks(d):
    if b - a > 2 * half + 1:
      y[a + half : b - half] = np.convolve(x[a:b], k, 'valid')
  return y


def shift(d, x, lag):
  """x delayed by lag samples (value from t - lag), within blocks."""
  y = np.full(len(x), np.nan)
  for a, b in blocks(d):
    if lag >= 0 and b - a > lag:
      y[a + lag : b] = x[a : b - lag]
    elif lag < 0 and b - a > -lag:
      y[a : b + lag] = x[a - lag : b]
  return y
