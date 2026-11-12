"""Scenario-specific metrics from closed_loop.py traces (nominal plant) and summary table."""
import argparse
import json
from pathlib import Path

import numpy as np


def settle_time(t, v, target, t0, tol=0.5, hold=2.0):
  """Seconds after t0 until |v - target| <= tol for `hold` seconds."""
  ok = np.abs(v - target) <= tol
  for i in np.flatnonzero(t >= t0):
    j = np.flatnonzero((t >= t[i]) & (t < t[i] + hold))
    if len(j) and ok[j].all():
      return float(t[i] - t0)
  return None


def metrics(scenario, rows):
  t, v, c = (np.array([r[k] for r in rows]) for k in ('t', 'v', 'cruise'))
  gap, pedal, act = (np.array([r[k] for r in rows]) for k in ('gap', 'pedal', 'active'))
  m = {}
  if np.isfinite(gap).any():
    head = gap / np.maximum(v, 1.0)
    m['min_gap_m'] = round(float(np.nanmin(gap)), 2)
    m['contact'] = bool(np.nanmin(gap) <= 0.0)
    m['time_headway_below_1s'] = round(float(np.sum(np.nan_to_num(head, nan=9) < 1.0) * 0.1), 1)
    m['mean_headway_s'] = round(float(np.nanmean(head[t > 5])), 2)
  if scenario in ('cruise_drop', 'speed_limit_drop'):
    t_down, t_up = 10.0, 35.0 if scenario == 'cruise_drop' else 40.0
    low = c[(t > t_down) & (t < t_up)][0]
    m['settle_down_s'] = settle_time(t, v, low, t_down)
    after = (t > t_down) & (t < t_up)
    m['undershoot_mps'] = round(float(max(0.0, np.max(low - v[after]))), 2)
    m['settle_up_s'] = settle_time(t, v, 22.0, t_up)
    m['overshoot_after_rise_mps'] = round(float(max(0.0, np.max(v[t > t_up] - 22.0))), 2)
  if scenario in ('steady', 'uphill', 'downhill', 'pitch_bias'):
    w = t > 10
    e = v[w] - c[w]
    m['rms_speed_error'] = round(float(np.sqrt(np.mean(e ** 2))), 3)
    m['max_abs_speed_error'] = round(float(np.max(np.abs(e))), 2)
  if scenario in ('driver_gas', 'brake_tap', 'shift_down'):
    t_rel = {'driver_gas': 14.0, 'brake_tap': 12.5, 'shift_down': 12.0}[scenario]
    w = t >= t_rel
    m['max_pedal_jump_after_reengage'] = round(float(np.max(np.abs(np.diff(pedal[w])), initial=0.0)), 2)
    m['max_underspeed_after'] = round(float(max(0.0, np.max(c[w] - v[w]))), 2)
    m['max_overspeed_after'] = round(float(max(0.0, np.max(v[w] - c[w]))), 2)
    m['settle_s'] = settle_time(t, v, 22.0, t_rel)
    m['command_while_inactive_max'] = float(np.max(np.array([r['command'] for r in rows])[~act], initial=0.0))
  m['pedal_total_variation'] = round(float(np.sum(np.abs(np.diff(pedal))) / 100.0), 2)
  return m


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--input', type=Path, required=True)
  p.add_argument('--output', type=Path, required=True)
  a = p.parse_args()
  out = {}
  for f in sorted(a.input.glob('cl-*.json')):
    _, rest = f.stem.split('-', 1)
    scenario, variant = rest.rsplit('-', 1)
    out.setdefault(scenario, {})[variant] = metrics(scenario, json.loads(f.read_text()))
  a.output.write_text(json.dumps(out, indent=1) + '\n')
  for s, vs in out.items():
    print(s)
    for v, m in vs.items():
      print('  ', v.ljust(13), m)


if __name__ == '__main__':
  main()
