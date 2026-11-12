"""Per-segment derive + lag-filtered pedal, decimated to 10 Hz rows for fitting/validation."""

import json
import sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import load
import dataset
import dyn

DYN = json.load(open(load.ROOT / 'analysis' / 'dynamics.json'))['best'][1:]
COLS = [
  't',
  'v',
  'a',
  'a_long',
  'f_spec',
  'grade_esp',
  'grade_pose',
  'pitch_valid',
  'ratio',
  'gear',
  'stable',
  'no_brake',
  'T',
  'T_lag',
  'pedal',
  'p_eff',
  'p_set',
  'N',
  'op_pedal',
  'driver_gas',
  'long_active',
  'cmd_pedal',
  'coast_s',
  'esp_ok',
  'lead',
  'drel',
  'vrel',
  'aTarget',
  'allow_throttle',
  'plan_a0',
  'out_accel',
  'v_cruise',
  'v_pid',
]


def one(path):
  dev, route, seg = path.parent.parent.name, path.parent.name, int(path.stem)
  f = np.load(path)
  if 'empty' in f.files:
    return None
  d = {k: f[k] for k in f.files if k != 'meta'}
  # Segments without the physical CAN channels (ignition-only, partial logs) carry no usable samples.
  if any(k not in d for k in ('esp_LONG_ACCEL', 'esp_CYL_PRES', 'brake_act', 'TQI', 'TQFR', 'v_wheel')):
    return None
  n = len(d['t'])
  for k in (
    'pitch',
    'pitch_valid',
    'carState.gasPressed',
    'cmd_enable',
    'cmd_pedal',
    'carControl.longActive',
    'radarState.leadOne.status',
    'radarState.leadOne.dRel',
    'radarState.leadOne.vRel',
    'longitudinalPlan.aTarget',
    'longitudinalPlan.allowThrottle',
    'plan_a0',
    'carControl.actuators.accel',
    'controlsState.vCruise',
    'controlsState.vPid',
    'esp_CYL_PRES',
    'brake_act',
  ):
    d.setdefault(k, np.full(n, np.nan))
  d['segment'] = np.full(n, seg)
  d['route'] = np.full(n, route, dtype=object)
  x = dataset.derive(d)
  p = x['pedal']
  pe = dyn.lag_filter(p, *DYN, d)
  coast = np.zeros(n)
  for a, b in load.blocks(d):
    c = 0.0
    for i in range(a, b):
      c = c + 0.02 if np.isfinite(p[i]) and p[i] < 0.5 else 0.0
      coast[i] = c
  # Causal, runtime-realisable ESC grade: LONG_ACCEL - carState.aEgo, 1 s first-order filter.
  d.setdefault('carState.aEgo', np.full(n, np.nan))
  raw = d['esp_LONG_ACCEL'] - d['carState.aEgo']
  g_c = np.full(n, np.nan)
  for a, b in load.blocks(d):
    s = np.nan
    for i in range(a, b):
      u = raw[i]
      if np.isfinite(u):
        s = u if not np.isfinite(s) else s + 0.02 / 1.02 * (u - s)
      g_c[i] = s
  # Codex map input: its pedal (0..1) averaged 0.4 s earlier over one second, as in its fit.
  pc = np.where(d['cmd_enable'] == 1, np.nan_to_num(d['cmd_pedal']), p / 100.0)
  p_codex = load.shift(d, load.smooth(d, pc, 25), 20)
  # Pedal settled within +-1 % for 1 s: steady-state calibration samples.
  pset = dyn.settled(d, p)
  rows = {
    't': d['t'],
    'v': d['v_wheel'],
    'a': x['a'],
    'a_long': load.deriv(d, d['v_wheel'], 25),
    'f_spec': x['f_spec'],
    'grade_esp': x['grade_esp'],
    'grade_pose': x['grade_pose'],
    'pitch_valid': d['pitch_valid'],
    'ratio': x['ratio'],
    'gear': x['gear'],
    'stable': x['stable'],
    'no_brake': x['no_brake'],
    'T': x['T'],
    'T_lag': load.shift(d, x['T'], 4),
    'pedal': p,
    'p_eff': pe,
    'p_set': pset,
    'N': d['N'],
    'op_pedal': d['cmd_enable'] == 1,
    'driver_gas': d['carState.gasPressed'] == 1,
    'long_active': d['carControl.longActive'] == 1,
    'cmd_pedal': d['cmd_pedal'],
    'coast_s': coast,
    'esp_ok': (d['esp_LONG_ACCEL_STAT'] == 0) & (d['esp_LONG_ACCEL_DIAG'] == 0),
    'lead': d['radarState.leadOne.status'] == 1,
    'drel': d['radarState.leadOne.dRel'],
    'vrel': d['radarState.leadOne.vRel'],
    'aTarget': d['longitudinalPlan.aTarget'],
    'allow_throttle': d['longitudinalPlan.allowThrottle'],
    'plan_a0': d['plan_a0'],
    'grade_causal': g_c,
    'p_codex': p_codex,
    'a_ego': d['carState.aEgo'],
    'out_accel': d['carControl.actuators.accel'],
    'v_cruise': d['controlsState.vCruise'],
    'v_pid': d['controlsState.vPid'],
  }
  keep = np.zeros(n, bool)
  keep[::5] = True
  keep &= np.isfinite(d['v_wheel']) & (d['v_wheel'] > 1.0)
  out = {k: np.asarray(v)[keep].astype(np.float32 if k != 't' else np.int64) for k, v in rows.items()}
  out['segment'] = np.full(keep.sum(), seg, np.int32)
  return dev, route, out


def main():
  files = sorted((load.ROOT / 'features').glob('*/*/*.npz'))
  acc = {}
  with ProcessPoolExecutor(int(sys.argv[1]) if len(sys.argv) > 1 else 6) as ex:
    for r in ex.map(one, files, chunksize=4):
      if r is None:
        continue
      dev, route, out = r
      acc.setdefault(dev, []).append((route, out))
  for dev, items in acc.items():
    cat = {k: np.concatenate([o[k] for _, o in items]) for k in items[0][1]}
    cat['route'] = np.concatenate([np.full(len(o['t']), r) for r, o in items])
    np.savez_compressed(load.ROOT / 'analysis' / f'rows_{dev}.npz', **cat)
    print(dev, len(cat['t']), 'rows', len(set(cat['route'])), 'routes', flush=True)


if __name__ == '__main__':
  main()
