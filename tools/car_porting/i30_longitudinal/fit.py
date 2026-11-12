"""Fit a compact monotone pedal/coast candidate from the audit's aligned CSV.

Run in the repository Docker environment. No fitting runs on the vehicle.
Example: python -m tools.car_porting.i30_longitudinal.fit --train samples.csv
  --validation other-drives.csv --output /audit/candidate
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

from openpilot.tools.car_porting.i30_longitudinal.lookup import PedalMap

PEDAL = np.array([0.0, 0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.0])
DELAY = 0.4  # Fixed from the earlier exploratory study, not tuned on validation.


def load_windows(path):
  rows = list(csv.DictReader(Path(path).open()))
  keys = [
    't',
    'v',
    'vraw',
    'pitch',
    'gear',
    'pedal',
    'active',
    'gear_valid',
    'brake',
    'driver_gas',
    'clutch',
    'can_valid',
    'valid_carState',
    'valid_carControl',
    'valid_carOutput',
    'valid_frogpilotCarState',
  ]
  data = {k: np.array([float(r[k]) if r[k] else np.nan for r in rows]) for k in keys}
  routes = np.array([r['route'] for r in rows])
  valid = np.all(np.isfinite(np.column_stack(list(data.values()))), axis=1)
  for k in ['active', 'gear_valid', 'can_valid', 'valid_carState', 'valid_carControl', 'valid_carOutput', 'valid_frogpilotCarState']:
    valid &= data[k] == 1
  for k in ['brake', 'driver_gas', 'clutch']:
    valid &= data[k] == 0
  valid &= (data['v'] > 8) & np.isin(data['gear'], [4, 5, 6]) & (data['pedal'] >= 0) & (data['pedal'] <= 1)
  windows = []
  for route in sorted(set(routes)):
    ids = np.flatnonzero(routes == route)
    # One-second windows, two-second exclusion after shifts/invalid/driver inputs.
    for i in range(ids[0] + 20, ids[-1] - 10, 10):
      history = slice(i - 20, i + 11)
      if not valid[history].all() or np.any(routes[history] != route) or np.ptp(data['gear'][history]) != 0:
        continue
      if np.any(np.abs(np.diff(data['t'][history]) - 0.1) > 0.025):
        continue
      delay = round(DELAY / 0.1)
      duration = np.mean(data['t'][i + 8 : i + 11]) - np.mean(data['t'][i - 1 : i + 2])
      a = (np.mean(data['vraw'][i + 8 : i + 11]) - np.mean(data['vraw'][i - 1 : i + 2])) / duration
      windows.append(
        dict(
          route=route,
          gear=int(data['gear'][i]),
          a=a,
          v=np.mean(data['v'][i : i + 10]),
          pitch=np.mean(data['pitch'][i : i + 10]),
          pedal=np.mean(data['pedal'][i - delay : i + 10 - delay]),
          coast=np.max(data['pedal'][history]) <= 0.001,
        )
      )
  if not windows:
    raise ValueError(f'No valid one-second windows in {path}')
  return {k: np.array([w[k] for w in windows]) for k in windows[0]}


def robust_fit(matrix, target, bounds, regularization=0.01):
  weights = np.ones(len(target))
  for _ in range(5):
    lhs = np.vstack([matrix * np.sqrt(weights)[:, None], np.eye(matrix.shape[1]) * regularization])
    rhs = np.r_[target * np.sqrt(weights), np.zeros(matrix.shape[1])]
    fit = lsq_linear(lhs, rhs, bounds=bounds, tol=1e-9)
    if not fit.success:
      raise RuntimeError(f'Fit failed: {fit.message}')
    weights = np.minimum(1, 0.15 / np.maximum(np.abs(target - matrix @ fit.x), 1e-9))
  return fit.x


def fit_map(w):
  coast = w['coast']
  for gear in (4, 5, 6):
    if sum(coast & (w['gear'] == gear)) < 20:
      raise ValueError(f'Insufficient coast windows for gear {gear}')
  v = (w['v'] - 17) / 10
  matrix = np.column_stack([col for g in (4, 5, 6) for col in ((w['gear'] == g), (w['gear'] == g) * v)] + [-9.81 * np.sin(w['pitch'])])
  # Flat-road coast is negative; speed coefficient nonpositive; grade gain 0–2.
  coefficients = robust_fit(matrix[coast], w['a'][coast], ([-2, -3] * 3 + [0], [0, 0] * 3 + [2]))
  result = dict(
    version=1,
    status='offline_candidate_not_for_control',
    delay=DELAY,
    min_inverse_slope=0.2,
    pedal=PEDAL.tolist(),
    pitch_gain=float(coefficients[-1]),
    max_abs_pitch=float(np.quantile(abs(w['pitch']), 0.99)),
    gears={},
  )
  for index, gear in enumerate((4, 5, 6)):
    mask = w['gear'] == gear
    speed = np.quantile(w['v'][mask], [0.05, 0.5, 0.95])
    if np.min(np.diff(speed)) <= 0.01:
      raise ValueError(f'Insufficient speed variation for gear {gear}')
    # Positive slopes at two speed endpoints guarantee monotonic pedal response.
    blend = np.clip((w['v'][mask] - speed[0]) / (speed[-1] - speed[0]), 0, 1)
    pedal_basis = np.clip(w['pedal'][mask, None] - PEDAL[:-1], 0, np.diff(PEDAL))
    matrix_pedal = np.column_stack([pedal_basis * (1 - blend[:, None]), pedal_basis * blend[:, None]])
    baseline = matrix[mask] @ coefficients
    slopes = robust_fit(matrix_pedal, w['a'][mask] - baseline, (0.01, 20), regularization=0.03).reshape(2, -1)
    table, supported, coverage = [], [], []
    for s in speed:
      weight = (s - speed[0]) / (speed[-1] - speed[0])
      gain = slopes[0] * (1 - weight) + slopes[1] * weight
      coast_flat = coefficients[index * 2] + coefficients[index * 2 + 1] * (s - 17) / 10
      table.append((coast_flat + np.r_[0, np.cumsum(gain * np.diff(PEDAL))]).tolist())
      row_support, row_counts = [], []
      for lo, hi in zip(PEDAL[:-1], PEDAL[1:], strict=True):
        cell = mask & (abs(w['v'] - s) <= 2.5) & (w['pedal'] >= lo) & (w['pedal'] < hi + 1e-9)
        count, routes = int(sum(cell)), len(set(w['route'][cell]))
        row_support.append(count >= 10 and routes >= 2)
        row_counts.append(dict(windows=count, routes=routes))
      supported.append(row_support)
      coverage.append(row_counts)
    result['gears'][str(gear)] = dict(speed=speed.tolist(), accel=table, supported=supported, coverage=coverage)
  return result


def score(calibration, windows):
  lookup = PedalMap(calibration)
  prediction = np.array(
    [
      lookup.acceleration(int(g), float(v), float(pitch), float(pedal))
      for g, v, pitch, pedal in zip(windows['gear'], windows['v'], windows['pitch'], windows['pedal'], strict=True)
    ],
    dtype=float,
  )

  inverse = np.array(
    [
      lookup.pedal(int(g), float(v), float(pitch), float(a))
      for g, v, pitch, a in zip(windows['gear'], windows['v'], windows['pitch'], windows['a'], strict=True)
    ],
    dtype=float,
  )

  def metrics(mask):
    supported = mask & np.isfinite(prediction)
    error = prediction[supported] - windows['a'][supported]
    return dict(
      windows=int(sum(mask)),
      supported_windows=int(sum(supported)),
      coverage=float(sum(supported) / max(1, sum(mask))),
      inverse_supported_windows=int(sum(mask & np.isfinite(inverse))),
      mae=float(np.mean(abs(error))) if len(error) else None,
      bias=float(np.mean(error)) if len(error) else None,
      p95_absolute_error=float(np.quantile(abs(error), 0.95)) if len(error) else None,
    )

  return dict(
    all=metrics(np.ones(len(prediction), dtype=bool)),
    coast=metrics(windows['coast']),
    gears={str(g): metrics(windows['gear'] == g) for g in (4, 5, 6)},
    routes={str(r): metrics(windows['route'] == r) for r in sorted(set(windows['route']))},
  )


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--train', type=Path, required=True)
  parser.add_argument('--validation', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  train, validation = load_windows(args.train), load_windows(args.validation)
  overlap = set(train['route']) & set(validation['route'])
  if overlap:
    raise ValueError(f'Training/validation routes overlap: {overlap}')
  calibration = fit_map(train)
  scores = dict(training=score(calibration, train), validation=score(calibration, validation))
  args.output.mkdir(parents=True, exist_ok=True)
  (args.output / 'pedal-map.json').write_text(json.dumps(calibration, indent=2, allow_nan=False) + '\n')
  (args.output / 'validation.json').write_text(json.dumps(scores, indent=2, allow_nan=False) + '\n')
  print(json.dumps(scores, indent=2))


if __name__ == '__main__':
  main()
