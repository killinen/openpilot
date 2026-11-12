"""Derived per-sample quantities and exclusion masks shared by fit and validation."""

import numpy as np
import load

G = 9.81
TQ_REF = 230.0  # EMS_DCT1.TQ_STND, constant 230 Nm in every log checked.
# Ratio clusters (rpm per m/s, wheel speed). Found from the ratio histogram of the
# data itself (see report); gears 4-6 match selfdrive/car/i30/gear.py.
CLUSTERS = {1: (430, 485), 2: (232, 265), 3: (162, 182), 4: (124, 137), 5: (99, 112), 6: (86, 97)}


def derive(d, smooth_half=12):
  v = d['v_wheel']
  out = {}
  out['a'] = load.deriv(d, v, smooth_half)  # wheel-speed acceleration (0.48 s centered)
  out['f_spec'] = load.smooth(d, d['esp_LONG_ACCEL'], smooth_half)  # specific force = a + g sin(grade)
  out['grade_esp'] = out['f_spec'] - out['a']
  out['grade_pose'] = G * np.sin(d['pitch']) if 'pitch' in d else np.full(len(v), np.nan)
  out['ratio'] = d['N'] / np.maximum(v, 0.1)
  out['T'] = d['TQI'] - d['TQFR']  # net torque, % of TQ_REF
  gear = np.zeros(len(v), int)
  for g, (lo, hi) in CLUSTERS.items():
    gear[(out['ratio'] >= lo) & (out['ratio'] <= hi)] = g
  out['gear'] = gear
  # Stable gear: same cluster for +-0.5 s (clutch engaged, no shift in progress).
  stable = np.zeros(len(v), bool)
  for a, b in load.blocks(d):
    gg = gear[a:b]
    if b - a < 51:
      continue
    same = np.ones(b - a, bool)
    for s in range(-25, 26, 5):
      sh = np.roll(gg, s)
      same &= sh == gg
    same[:25] = False
    same[-25:] = False
    stable[a:b] = same & (gg > 0)
  out['stable'] = stable
  brake = (d['brake_act'] == 2) | (d['esp_CYL_PRES'] > 3.0) | ~np.isfinite(d['esp_CYL_PRES']) | ~np.isfinite(d['brake_act'])
  # Exclude 1 s around any brake application (or unknown brake state).
  out['no_brake'] = ~dilate(d, brake, 50)
  driver = d['carState.gasPressed'] == 1 if 'carState.gasPressed' in d else np.zeros(len(v), bool)
  out['op_pedal'] = (d.get('cmd_enable', np.zeros(len(v))) == 1) & ~driver
  out['pedal'] = d['PV_AV_CAN']
  return out


def dilate(d, mask, half):
  m = mask.astype(float)
  out = np.zeros(len(m), bool)
  k = np.ones(2 * half + 1)
  for a, b in load.blocks(d):
    out[a:b] = np.convolve(m[a:b], k, 'same') > 0
  return out


def base_mask(d, x):
  return (
    x['stable']
    & x['no_brake']
    & (d['v_wheel'] > 2.0)
    & np.isfinite(x['a'])
    & np.isfinite(x['f_spec'])
    & np.isfinite(x['T'])
    & np.isfinite(x['pedal'])
    & (d['esp_LONG_ACCEL_STAT'] == 0)
    & (d['esp_LONG_ACCEL_DIAG'] == 0)
  )
