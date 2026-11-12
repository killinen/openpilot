

"""Pedal -> engine torque dynamics: dead time + asymmetric first-order lag.

Fits a steady engine table from settled-pedal samples, then grid-searches the
lag that best predicts logged net torque from logged pedal on 50 Hz data.
"""
import json
import sys
import numpy as np
import load
import dataset
import table

DT = 0.02


def lag_filter(p, dead, tau_up, tau_down, d):
  y = np.full(len(p), np.nan)
  k = int(round(dead / DT))
  au, ad = DT / (tau_up + DT), DT / (tau_down + DT)
  for a, b in load.blocks(d):
    x = p[a:b]
    if b - a <= k + 1 or not np.isfinite(x).all():
      # Run filter only over finite stretches.
      pass
    out = np.full(b - a, np.nan)
    s = np.nan
    for i in range(k, b - a):
      u = x[i - k]
      if not np.isfinite(u):
        s = np.nan
        continue
      if not np.isfinite(s):
        s = u
      else:
        s += (au if u > s else ad) * (u - s)
      out[i] = s
    y[a:b] = out
  return y


def settled(d, p, half=50, tol=1.0):
  """Pedal within +-tol % over the previous `half` samples."""
  ok = np.zeros(len(p), bool)
  for a, b in load.blocks(d):
    x = p[a:b]
    if b - a <= half:
      continue
    win = np.lib.stride_tricks.sliding_window_view(x, half + 1)
    rng = np.nanmax(win, 1) - np.nanmin(win, 1)
    ok[a + half : b] = rng <= 2 * tol
  return ok


def main(devices, max_rows=3_000_000):
  d = load.load(devices=devices)
  x = dataset.derive(d)
  p, n, T = x['pedal'], d['N'], x['T']
  base = np.isfinite(p) & np.isfinite(n) & np.isfinite(T) & (n > 700) & (d['v_wheel'] > 2) & x['stable']
  st = base & settled(d, p)
  idx = np.flatnonzero(st)[::5]
  tab, sup = table.fit_engine(p[idx], n[idx], T[idx])
  print('steady rows', len(idx))
  res = {}
  evalmask = base & ~st  # transients matter most
  ev = np.flatnonzero(evalmask)
  print('transient rows', len(ev))
  best = None
  for dead in (0.0, 0.04, 0.08, 0.12, 0.16):
    for tu in (0.05, 0.15, 0.25, 0.35, 0.5, 0.7):
      for td in (0.05, 0.1, 0.2, 0.35):
        pf = lag_filter(p, dead, tu, td, d)
        k = ev[np.isfinite(pf[ev])]
        e = table.engine_torque(tab, pf[k], n[k]) - T[k]
        mae = float(np.mean(np.abs(e)))
        res[f'{dead},{tu},{td}'] = mae
        if best is None or mae < best[0]:
          best = (mae, dead, tu, td)
        print(dead, tu, td, round(mae, 2), flush=True)
  e0 = table.engine_torque(tab, p[ev], n[ev]) - T[ev]
  print('no-dynamics transient MAE', np.mean(np.abs(e0)), 'best', best)
  json.dump(
    dict(best=best, static_mae=float(np.mean(np.abs(e0))), grid=res, steady_rows=len(idx), transient_rows=len(ev)),
    open(load.ROOT / 'analysis' / 'dynamics.json', 'w'),
    indent=1,
  )


if __name__ == '__main__':
  main(sys.argv[1:] or ['ff528bde139e99de'])  # i30 runtime device only
