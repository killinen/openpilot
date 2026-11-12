# ruff: noqa: B905  (host analysis runs Python 3.8, which has no zip(strict=))
import os
from pathlib import Path

"""Build an aligned 50 Hz feature table per segment from converted npz files (numpy only).

Every channel is backward-aligned (latest sample at or before the grid time) with a
maximum age, so no future information leaks into a sample. Provenance columns
(device, route, segment) are kept on every row.
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np

ROOT = Path(os.environ['I30_LONG_DATA'])  # data directory outside the repository (required)
DT = 0.02
KPH = 1 / 3.6


def pick(d, *names):
  for n in names:
    if n in d.files:
      return n
  return None


def align(grid, t, x, max_age):
  i = np.searchsorted(t, grid, side='right') - 1
  ok = i >= 0
  i = np.clip(i, 0, len(t) - 1)
  out = x[i].astype(np.float64)
  ok &= (grid - t[i]) <= max_age * 1e9
  out[~ok] = np.nan
  return out


def segment_features(path):
  d = np.load(path, allow_pickle=False)
  meta = json.loads(str(d['meta']))
  ems = pick(d, 'can.EMS_DCT1.0.t')
  whl = pick(d, 'can.TCS5.0.t')
  if ems is None or whl is None:
    return None, meta
  t_ems = d[ems]
  grid = np.arange(t_ems[0], t_ems[-1], int(DT * 1e9), dtype=np.int64)
  if len(grid) < 100:
    return None, meta
  f = {'t': grid}
  for k in ('PV_AV_CAN', 'N', 'TQI', 'TQI_ACOR', 'TQFR', 'TQ_STND'):
    f[k] = align(grid, t_ems, d['can.EMS_DCT1.0.' + k], 0.05)
  wt = d[whl]
  wheels = np.vstack([d['can.TCS5.0.WHEEL_' + w] for w in ('FL', 'FR', 'RL', 'RR')])
  f['v_front'] = align(grid, wt, wheels[:2].mean(0) * KPH, 0.06)
  f['v_rear'] = align(grid, wt, wheels[2:].mean(0) * KPH, 0.06)
  f['v_wheel'] = align(grid, wt, wheels.mean(0) * KPH, 0.06)
  if pick(d, 'can.ESP2.0.t'):
    for k in ('LONG_ACCEL', 'LONG_ACCEL_STAT', 'LONG_ACCEL_DIAG', 'CYL_PRES'):
      f['esp_' + k] = align(grid, d['can.ESP2.0.t'], d['can.ESP2.0.' + k], 0.05)
  if pick(d, 'can.EMS_DCT2.0.t'):
    f['brake_act'] = align(grid, d['can.EMS_DCT2.0.t'], d['can.EMS_DCT2.0.BRAKE_ACT'], 0.05)
  if pick(d, 'can.EMS2.0.t'):
    for k in ('TPS', 'CLU_ACK'):
      f['ems2_' + k] = align(grid, d['can.EMS2.0.t'], d['can.EMS2.0.' + k], 0.05)
  if pick(d, 'can.EMS6.0.t'):
    for k in ('TQI_TARGET', 'TQI_MAX', 'CF_Ems_AclAct', 'CRUISE_LAMP_S'):
      f['ems6_' + k] = align(grid, d['can.EMS6.0.t'], d['can.EMS6.0.' + k], 0.05)
  if pick(d, 'can.CLU2.0.t'):
    f['reverse'] = align(grid, d['can.CLU2.0.t'], d['can.CLU2.0.CF_Clu_SwiGearR'], 0.3)
  sensor = pick(d, *[f'can.GAS_SENSOR.{s}.t' for s in (1, 2, 0, 129)])
  if sensor:
    p = sensor[:-2]
    f['gas_sensor'] = align(grid, d[sensor], (d[p + '.INTERCEPTOR_GAS'] + d[p + '.INTERCEPTOR_GAS2']) / 2, 0.06)
    f['gas_sensor_state'] = align(grid, d[sensor], d[p + '.STATE'], 0.06)
  # Outgoing command as actually transmitted (sendcan), else bus echo.
  cmd = pick(d, *[f'sendcan.GAS_COMMAND.{s}.t' for s in (1, 2, 0)], *[f'can.GAS_COMMAND.{s}.t' for s in (129, 130, 128)])
  if cmd:
    p = cmd[:-2]
    en = d[p + '.ENABLE']
    f['cmd_enable'] = align(grid, d[cmd], en, 0.06)
    f['cmd_pedal'] = align(grid, d[cmd], np.where(en > 0, d[p + '.GAS_COMMAND'] / 100.0, 0.0), 0.06)
  for ch, fields, age in (
    ('carState', ('vEgo', 'aEgo', 'gas', 'gasPressed', 'brakePressed', 'clutchPressed', 'engineRpm', 'canValid'), 0.05),
    ('carControl', ('enabled', 'longActive', 'actuators.accel'), 0.05),
    ('carOutput', ('actuatorsOutput.gas', 'actuatorsOutput.accel'), 0.05),
    ('controlsState', ('enabled', 'longControlState', 'vCruise', 'vPid', 'upAccelCmd', 'uiAccelCmd', 'ufAccelCmd', 'forceDecel'), 0.05),
    ('longitudinalPlan', ('aTarget', 'allowThrottle', 'hasLead'), 0.25),
    ('radarState', ('leadOne.status', 'leadOne.dRel', 'leadOne.vRel', 'leadOne.vLead', 'leadOne.aLeadK', 'leadOne.modelProb'), 0.25),
    ('frogpilotPlan', ('vCruise', 'maxAcceleration', 'minAcceleration', 'disableThrottle'), 0.25),
  ):
    if ch + '.t' not in d.files:
      continue
    for k in fields:
      if f'{ch}.{k}' in d.files and d[f'{ch}.{k}'].dtype.kind == 'f' and d[f'{ch}.{k}'].ndim == 1:
        f[f'{ch}.{k}'] = align(grid, d[ch + '.t'], d[f'{ch}.{k}'], age)
  # Pitch exactly as the controller sees it: carControl.orientationNED (copied by controlsd from
  # liveLocationKalman.calibratedOrientationNED). livePose differs by a constant ~0.011 rad on the C3X.
  if 'carControl.orientationNED' in d.files and d['carControl.orientationNED'].ndim == 2:
    co = d['carControl.orientationNED'][:, 1]
    f['pitch'] = align(grid, d['carControl.t'], co, 0.05)
    valid = np.isfinite(f['pitch'])
    # controlsd copies the value without checking it; an uninitialized liveLocationKalman gives a frozen ~1e-9 pitch.
    if 'liveLocationKalman.calibratedOrientationNED.valid' in d.files:
      llk_ok = align(grid, d['liveLocationKalman.t'], d['liveLocationKalman.calibratedOrientationNED.valid'], 0.25)
      valid &= np.nan_to_num(llk_ok) == 1
    f['pitch_valid'] = valid.astype(float)
  elif 'liveLocationKalman.calibratedOrientationNED.value' in d.files and d['liveLocationKalman.calibratedOrientationNED.value'].ndim == 2:
    v = d['liveLocationKalman.calibratedOrientationNED.value']
    f['pitch'] = align(grid, d['liveLocationKalman.t'], v[:, 1], 0.25)
    f['pitch_valid'] = align(grid, d['liveLocationKalman.t'], d['liveLocationKalman.calibratedOrientationNED.valid'], 0.25)
  if 'livePose.orientationNED.y' in d.files:
    f['pitch_pose'] = align(grid, d['livePose.t'], d['livePose.orientationNED.y'], 0.25)
  if 'longitudinalPlan.accels' in d.files and d['longitudinalPlan.accels'].ndim == 2:
    acc = d['longitudinalPlan.accels']
    f['plan_a0'] = align(grid, d['longitudinalPlan.t'], acc[:, 0], 0.25)
    f['plan_a_min'] = align(grid, d['longitudinalPlan.t'], np.nanmin(acc, axis=1), 0.25)
    f['plan_a_max'] = align(grid, d['longitudinalPlan.t'], np.nanmax(acc, axis=1), 0.25)
  return f, meta


def main():
  out_dir = ROOT / 'features'
  files = sorted((ROOT / 'raw').glob('*/*/*.npz'))
  todo = [p for p in files if not (out_dir / p.parent.parent.name / p.parent.name / p.name).exists()]

  with ProcessPoolExecutor(int(sys.argv[1]) if len(sys.argv) > 1 else 6) as ex:
    for p, (f, meta) in zip(todo, ex.map(segment_features, todo, chunksize=2)):
      dest = out_dir / p.parent.parent.name / p.parent.name / p.name
      dest.parent.mkdir(parents=True, exist_ok=True)
      if f is None:
        np.savez(dest, meta=np.array(json.dumps(meta)), empty=np.array(True))
      else:
        np.savez_compressed(dest, meta=np.array(json.dumps(meta)), **f)


if __name__ == '__main__':
  main()
