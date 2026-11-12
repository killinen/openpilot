# ruff: noqa: B905  (host analysis runs Python 3.8, which has no zip(strict=))
"""Export analysis/model.json to the runtime calibration selfdrive/car/i30/pedal_map.json."""

import os
from pathlib import Path
import json
import subprocess
import sys
import numpy as np
import load

REPO = os.environ.get('I30_REPO', str(Path(__file__).resolve().parents[4]))
SUPPORT_MIN = 30  # 10 Hz settled-pedal samples (bilinear weight) per knot
MIN_SLOPE = 0.25  # % torque per % pedal: below this the inverse is ill-conditioned
CONTROLLABLE = {4: (130.6, 10.0), 5: (105.7, 8.0), 6: (92.1, 6.0)}  # gear.py windows (clutch-release tolerance 0.8x below)
RELEASE_TOL = 0.8
KNEE = 2.0  # % torque


def main(out_path, model_path=None):
  m = json.load(open(model_path or load.ROOT / 'analysis' / 'model.json'))
  ratios = json.load(open(load.ROOT / 'analysis' / 'gear_ratios.json')) if (load.ROOT / 'analysis' / 'gear_ratios.json').exists() else {}
  pb = np.array(m['pedal_bp'])
  T = np.array(m['torque'])
  S = np.array(m['support'])
  knot_ok = S >= SUPPORT_MIN
  cell_ok = knot_ok[:, :-1] & knot_ok[:, 1:]
  max_pedal, saturates = [], []
  for r in range(len(T)):
    row = T[r].copy()
    for j in range(1, len(row)):
      row[j] = max(row[j], row[j - 1] + MIN_SLOPE * (pb[j] - pb[j - 1]))
    T[r] = row
    # Supported prefix of the row, then the knee: first pedal reaching within KNEE % torque
    # of the supported maximum (beyond it extra pedal gives no steady torque).
    j = 0
    while j < len(pb) - 1 and cell_ok[r, j]:
      j += 1
    raw = np.array(m['torque'][r])
    knee = False
    if j > 0:
      top = raw[: j + 1].max()
      jk = int(np.flatnonzero(raw[: j + 1] >= top - KNEE)[0])
      knee = jk < j  # supported cells beyond jk add no torque: genuine saturation
      j = jk
    saturates.append(bool(knee))
    max_pedal.append(float(pb[j]))
  gears = {}
  for g in range(1, 7):
    k = m['k'].get(str(g))
    if k is None:
      continue
    if g in CONTROLLABLE:
      ratio, tol = CONTROLLABLE[g]
      gears[str(g)] = dict(ratio=ratio, tol=tol * RELEASE_TOL, k=k, controllable=True)
    elif str(g) in ratios:
      gears[str(g)] = dict(
        ratio=ratios[str(g)]['median'],
        tol=ratios[str(g)]['tol'],
        k=k,
        controllable=False,
        note='calibrated offline; clutch detection disengages outside gears 4-6',
      )
  try:
    commit = subprocess.check_output(['git', '-C', REPO, 'rev-parse', 'HEAD'], text=True).strip()
  except Exception:
    commit = None
  cal = dict(
    version=2,
    status='fitted offline; see tools/car_porting/i30_longitudinal/README.md',
    source=dict(
      fit_commit=commit,
      train_routes=len(m['train_routes']),
      holdout_routes=len(m['holdout_routes']),
      n_engine=m['n_engine'],
      n_vehicle=m['n_vehicle'],
      n_grade=m['n_grade'],
    ),
    pedal_bp=pb.tolist(),
    rpm_bp=m['rpm_bp'],
    torque=np.round(T, 3).tolist(),
    max_pedal=max_pedal,
    saturates=saturates,
    supported=cell_ok.tolist(),
    support_min=SUPPORT_MIN,
    min_slope=MIN_SLOPE,
    gears=gears,
    drag=dict(c0=round(m['c0'], 5), c2=round(m['c2'], 7)),
    grade=dict(gain=round(m['grade_gain'], 4), offset=round(m['grade_offset'], 4), max_abs_pitch=0.09, pitch_source='carControl.orientationNED'),
    pedal_command=m.get('pedal_command', dict(scale=1.03, offset=0.55)),
    limits=dict(v_min=m.get('v_min', 5.0), v_max=m.get('v_max', 40.0)),
    dynamics=dict(dead_time=m['dynamics'][0], tau_up=m['dynamics'][1], tau_down=m['dynamics'][2], torque_to_accel_delay=0.08),
  )
  open(out_path, 'w').write(json.dumps(cal, indent=1) + '\n')
  print('wrote', out_path, 'max_pedal by rpm', dict(zip(m['rpm_bp'], max_pedal)))


if __name__ == '__main__':
  main(sys.argv[1] if len(sys.argv) > 1 else f'{REPO}/selfdrive/car/i30/pedal_map.json', sys.argv[2] if len(sys.argv) > 2 else None)
