"""Markdown summary of closed_loop.py results (nominal-plant trace metrics + worst case over perturbed plants)."""
import argparse
import json
from pathlib import Path

from openpilot.tools.car_porting.i30_longitudinal.closed_loop_metrics import metrics

VARIANTS = ['legacy', 'map', 'map+lift', 'map+lift+gap']


def fmt(x, nd=1):
  return '–' if x is None else (f'{x:.{nd}f}' if isinstance(x, float) else str(x))


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--clean', type=Path, required=True)
  p.add_argument('--noise', type=Path, required=True)
  a = p.parse_args()
  out = {}
  lines = ['| Scenario (metric) | ' + ' | '.join(VARIANTS) + ' |', '|---|' + '---:|' * len(VARIANTS)]
  for label, d in (('', a.clean), (' + lead noise', a.noise)):
    allres = json.loads((d / 'closed-loop.json').read_text())
    scen = sorted({r['scenario'] for r in allres}, key=lambda s: [r['scenario'] for r in allres].index(s))
    for s in scen:
      row = {}
      for v in VARIANTS:
        f = d / f'cl-{s}-{v}.json'
        if f.exists():
          row[v] = metrics(s, json.loads(f.read_text()))
          worst = [r['min_gap'] for r in allres if r['scenario'] == s and r['variant'] == v and r['min_gap'] is not None]
          row[v]['worst_min_gap_over_plants'] = min(worst) if worst else None
      out[s + label] = row
      m0 = row.get('legacy', {})
      if 'min_gap_m' in m0:
        keys = [('min_gap_m', 'min gap m'), ('worst_min_gap_over_plants', 'worst over plants'), ('mean_headway_s', 'mean headway s')]
      elif 'settle_down_s' in m0:
        keys = [('settle_down_s', 'settle to lower target s'), ('undershoot_mps', 'undershoot m/s'), ('settle_up_s', 'settle back s')]
      elif 'rms_speed_error' in m0:
        keys = [('rms_speed_error', 'RMS speed error m/s'), ('max_abs_speed_error', 'max |error| m/s')]
      elif 'settle_s' in m0:
        keys = [('settle_s', 'settle after release/re-engage s'), ('max_underspeed_after', 'max underspeed m/s'), ('command_while_inactive_max', 'max command while inactive')]
      else:
        keys = []
      for k, name in keys:
        lines.append(f'| {s}{label} ({name}) | ' + ' | '.join(fmt(row.get(v, {}).get(k), 2) for v in VARIANTS) + ' |')
  (a.clean / 'summary.json').write_text(json.dumps(out, indent=1) + '\n')
  print('\n'.join(lines))


if __name__ == '__main__':
  main()
