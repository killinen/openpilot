"""Offline observation of a coast-state gap candidate; not closed-loop replay."""

import argparse
import csv
import json
from pathlib import Path
from timeit import timeit

import numpy as np

from openpilot.tools.car_porting.i30_longitudinal.lookup import PedalMap, coast_state_candidate


def evaluate(samples, calibration):
  lookup = PedalMap(calibration)
  totals = dict(
    valid_coast_seconds=0.0,
    covered_coast_seconds=0.0,
    eligible_seconds=0.0,
    speed_state_changed_seconds=0.0,
    accel_state_changed_seconds=0.0,
    naive_feedforward_positive_pedal_seconds=0.0,
  )
  gaps, candidates = [], []
  previous = None
  coast_duration = 0.0
  for row in csv.DictReader(Path(samples).open()):

    def val(key, row=row):
      return float(row[key]) if row.get(key) else float('nan')

    t, gear = val('t'), val('gear')
    dt = t - previous[1] if previous is not None and row['route'] == previous[0] and gear == previous[2] else 0.0
    continuity = 0 < dt <= 0.15
    dt = dt if continuity else 0.0
    previous = row['route'], t, gear
    valid = (
      val('active') == 1
      and val('gear_valid') == 1
      and val('can_valid') == 1
      and val('v') > 8
      and all(val(k) == 0 for k in ['brake', 'clutch', 'driver_gas'])
      and all(
        val('valid_' + k) == 1 for k in ['carState', 'carControl', 'carOutput', 'frogpilotCarState', 'frogpilotPlan', 'longitudinalPlan', 'controlsState']
      )
      and val('pedal') <= 0.001
    )
    coast_duration = coast_duration + dt if valid and continuity else 0.0
    if not valid or coast_duration < 0.5:
      continue
    totals['valid_coast_seconds'] += dt
    coast = lookup.acceleration(int(gear), val('v'), val('pitch'), 0.0)
    if coast is None:
      continue
    totals['covered_coast_seconds'] += dt
    # Missing forceDecel must disable the candidate, not silently assume false.
    force_decel = val('force_decel') != 0
    speed, accel = coast_state_candidate(val('plan_v0'), val('plan_a0'), val('v'), coast, coast_duration, True, val('plan_cruise'), force_decel=force_decel)
    eligible = not force_decel and val('plan_cruise') >= val('v') - 0.5
    if eligible:
      totals['eligible_seconds'] += dt
      gaps.append([val('v') - val('plan_v0'), coast - val('plan_a0')])
    totals['speed_state_changed_seconds'] += dt * (speed > val('plan_v0') + 1e-6)
    totals['accel_state_changed_seconds'] += dt * (accel > val('plan_a0') + 1e-6)
    raw = val('p') + val('i') + val('f')
    naive = val('p') + val('i') + max(val('f'), coast - 0.15)
    # Diagnostic only: logged P/I/FF are asynchronous; this is not counterfactual vehicle motion.
    totals['naive_feedforward_positive_pedal_seconds'] += dt * (val('requested') <= 0 and raw <= 0 and naive > 0.05)
    if eligible and (speed > val('plan_v0') + 0.01 or accel > val('plan_a0') + 0.01):
      candidates.append(
        dict(route=row['route'], t=t, gear=gear, v=val('v'), coast=coast, old_speed=val('plan_v0'), old_accel=val('plan_a0'), speed=speed, accel=accel)
      )
  gaps = np.array(gaps)
  result = dict(
    seconds={k: round(v, 3) for k, v in totals.items()},
    eligible_gap_p05_p50_p95=np.quantile(gaps, [0.05, 0.5, 0.95], axis=0).tolist() if len(gaps) else [],
    meaning='Initial-state shadow calculation only; no rerun of MPC or simulated lead following.',
  )
  # Desktop measurement includes object access and interpolation; no claim about C3X timing.
  result['desktop_microseconds_per_inverse_lookup'] = timeit(lambda: lookup.pedal(5, 15.0, 0.0, 0.1), number=100000) * 10
  return result, candidates


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--samples', type=Path, required=True)
  parser.add_argument('--calibration', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  result, candidates = evaluate(args.samples, json.loads(args.calibration.read_text()))
  args.output.mkdir(parents=True, exist_ok=True)
  (args.output / 'gap-evaluation.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
  if candidates:
    with (args.output / 'gap-candidates.csv').open('w') as f:
      writer = csv.DictWriter(f, fieldnames=list(candidates[0]))
      writer.writeheader()
      writer.writerows(candidates)
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
