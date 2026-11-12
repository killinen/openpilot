"""Inverse-map validation with the runtime class on withheld drives.

For settled-pedal samples (pedal within +-1 % for 1 s, clutch engaged, no brake), the
measured 1 s acceleration and the logged pedal form a steady operating point. The runtime
map's command for that acceleration is compared with the logged ECU pedal (converted to
command units). Also reports coverage, fallback reasons and conditioning.
"""

import json
import sys
import numpy as np
import load
from fit_model import load_rows, holdout, C3X
from lift_replay import map_mod


def main():
  d = load_rows()
  for k in ('stable', 'no_brake', 'p_set', 'op_pedal', 'driver_gas'):
    d[k] = np.nan_to_num(d[k]).astype(bool)
  hold = np.array([holdout(r) for r in d['route']])
  # Optional: alternative calibration file and output name, to score maps on identical samples.
  m = map_mod.I30LongitudinalMap(json.load(open(sys.argv[1])) if len(sys.argv) > 1 else None)
  base = hold & d['stable'] & d['no_brake'] & d['p_set'] & np.isfinite(d['a_long']) & np.isfinite(d['grade_pose']) & (d['v'] > 5)
  base &= d['device'] == C3X  # engine table applies to the interceptor-fitted configuration
  idx = np.flatnonzero(base)
  pitch = np.arcsin(np.clip(d['grade_pose'] / 9.81, -1, 1))
  got, want, slope, reason = [], [], [], {}
  for i in idx:
    a, v, n, p = float(d['a_long'][i]), float(d['v'][i]), float(d['N'][i]), float(pitch[i])
    c = m.command(a, v, n, p)
    if c is None:
      key = 'gear' if m.gear_gain(v, n) is None else ('rpm' if m._row(n) is None else 'pedal_coverage')
      reason[key] = reason.get(key, 0) + 1
      got.append(np.nan)
      want.append(np.nan)
      slope.append(np.nan)
      continue
    c2 = m.command(a + 0.1, v, n, p)
    got.append(c * 100)
    want.append(m.ecu_to_command(float(d['pedal'][i])) * 100)
    slope.append((c2 - c) * 100 if c2 is not None else np.nan)
  got, want, slope = map(np.array, (got, want, slope))
  ok = np.isfinite(got)
  err = got[ok] - want[ok]
  gear = d['gear'][idx][ok]
  res = dict(
    samples=int(len(idx)),
    covered=int(ok.sum()),
    coverage=round(float(ok.mean()), 3),
    fallback_reasons=reason,
    mae_pedal_pct=round(float(np.mean(np.abs(err))), 2),
    bias_pedal_pct=round(float(np.mean(err)), 2),
    p95_pedal_pct=round(float(np.percentile(np.abs(err), 95)), 2),
    pedal_change_per_0p1_mps2_p50_p95=np.nanpercentile(slope[ok], [50, 95]).round(2).tolist(),
    zero_pedal_samples=int(np.sum(want[ok] == 0)),
    zero_pedal_predicted_zero=int(np.sum((want[ok] == 0) & (got[ok] == 0))),
    by_gear={
      int(g): dict(n=int(np.sum(gear == g)), mae=round(float(np.mean(np.abs(err[gear == g]))), 2), bias=round(float(np.mean(err[gear == g])), 2))
      for g in np.unique(gear)
    },
  )
  # Legacy linear conversion implied by the same operating point: pedal = accel/1.6 (no road load).
  legacy = np.clip(d['a_long'][idx][ok] / 1.6, 0, 1) * 100
  res['legacy_static_mae_pedal_pct'] = round(float(np.mean(np.abs(legacy - want[ok]))), 2)
  (load.ROOT / 'analysis' / (sys.argv[2] if len(sys.argv) > 2 else 'inverse-validation.json')).write_text(json.dumps(res, indent=1))
  print(json.dumps(res, indent=1))


if __name__ == '__main__':
  main()
