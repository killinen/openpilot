# ruff: noqa: B905  (host analysis runs Python 3.8, which has no zip(strict=))
"""Coverage of the fetched data (C3X engine-table configuration and all devices for vehicle law)."""

import json
import numpy as np
import load
from fit_model import load_rows, C3X


def main():
  d = load_rows()
  for k in ('stable', 'no_brake', 'long_active', 'lead'):
    d[k] = np.nan_to_num(d[k]).astype(bool)
  inv = json.load(open(load.ROOT / 'segment-inventory.json'))
  fetched = {(p.parent.parent.name, p.parent.name, int(p.name.split('.')[0])) for p in (load.ROOT / 'raw').glob('*/*/*.jsonl.gz')}
  per_dev = {}
  for dev in sorted({j[0] for j in inv}):
    tot = [j for j in inv if j[0] == dev]
    got = [j for j in tot if tuple(j) in fetched]
    per_dev[dev] = dict(
      segments_available=len(tot), segments_fetched=len(got), routes_available=len({j[1] for j in tot}), routes_with_data=len({j[1] for j in got})
    )
  out = dict(inventory=per_dev)
  c3 = (d['device'] == C3X) & d['stable'] & d['no_brake']
  out['c3x_hours'] = dict(
    moving_stable_clutch=round(float(c3.sum() * 0.1 / 3600), 2),
    engaged=round(float((c3 & d['long_active']).sum() * 0.1 / 3600), 2),
    engaged_following=round(float((c3 & d['long_active'] & d['lead']).sum() * 0.1 / 3600), 2),
  )
  vb = [0, 5, 10, 15, 20, 25, 30, 45]
  pb = [0, 0.5, 10, 25, 40, 60, 101]
  table = {}
  for g in range(1, 7):
    gm = c3 & (d['gear'] == g)
    table[g] = dict(
      minutes=round(float(gm.sum() * 0.1 / 60), 1),
      by_speed={f'{lo}-{hi}': round(float((gm & (d['v'] >= lo) & (d['v'] < hi)).sum() * 0.1 / 60), 1) for lo, hi in zip(vb[:-1], vb[1:])},
      by_pedal={f'{lo}-{hi}': round(float((gm & (d['pedal'] >= lo) & (d['pedal'] < hi)).sum() * 0.1 / 60), 1) for lo, hi in zip(pb[:-1], pb[1:])},
      coast_1s_minutes=round(float((gm & (d['coast_s'] >= 1)).sum() * 0.1 / 60), 1),
      rpm_p5_p95=np.percentile(d['N'][gm], [5, 95]).round(0).tolist() if gm.sum() > 50 else None,
    )
  out['c3x_minutes_by_gear'] = table
  gr = d['grade_pose'][c3 & np.isfinite(d['grade_pose'])] / 9.81 * 100
  out['c3x_grade_percent_p1_p5_p50_p95_p99'] = np.percentile(gr, [1, 5, 50, 95, 99]).round(2).tolist()
  out['c3x_minutes_abs_grade_over_4pct'] = round(float(np.sum(np.abs(gr) > 4) * 0.1 / 60), 1)
  (load.ROOT / 'analysis' / 'coverage.json').write_text(json.dumps(out, indent=1))
  print(json.dumps(out, indent=1))


if __name__ == '__main__':
  main()
