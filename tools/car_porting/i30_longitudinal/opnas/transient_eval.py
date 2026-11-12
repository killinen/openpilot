"""5 s open-loop speed prediction from logged pedal on withheld C3X drives (transient check)."""

import json
import numpy as np
import os
from pathlib import Path

REPO = os.environ.get('I30_REPO', str(Path(__file__).resolve().parents[4]))
import load
import dataset
import dyn
import table
from fit_model import holdout, PedalMap, C3X

H = 250  # 5 s at 50 Hz


def main():
  model = json.load(open(load.ROOT / 'analysis' / 'model.json'))
  codex = PedalMap(json.load(open(REPO + '/tools/car_porting/i30_longitudinal/results/2026-11-12/pedal-map.json')))
  routes = sorted({p.parent.name for p in (load.ROOT / 'features' / C3X).glob('*/*.npz') if holdout(p.parent.name)})
  tab = np.array(model['torque'])
  k = {int(g): v for g, v in model['k'].items()}
  errs = {'physical_dynamic': [], 'physical_static': [], 'codex_map': [], 'constant_accel': []}
  starts = 0
  for route in routes:
    d = load.load(devices=[C3X], routes=[route])
    if not len(d.get('t', [])):
      continue
    x = dataset.derive(d)
    pe = dyn.lag_filter(x['pedal'], *model['dynamics'], d)
    pitch = d['pitch']
    pc = np.where(d.get('cmd_enable', np.zeros(len(pitch))) == 1, np.nan_to_num(d.get('cmd_pedal', np.zeros(len(pitch)))), x['pedal'] / 100)
    ok = x['stable'] & x['no_brake'] & np.isfinite(pe) & np.isfinite(pitch) & (d['v_wheel'] > 5)
    for a, b in load.blocks(d):
      for s in range(a + 50, b - H, 500):  # one window every 10 s
        w = slice(s, s + H)
        if not ok[w].all() or len(set(x['gear'][w])) != 1:
          continue
        g = int(x['gear'][s])
        if g not in k:
          continue
        starts += 1
        v0, v_true = d['v_wheel'][s], d['v_wheel'][s + H - 1]
        n = d['N'][w]
        ratio = x['ratio'][w]
        v = d['v_wheel'][w]
        res = model['c0'] + model['c2'] * v**2 + model['grade_gain'] * 9.81 * np.sin(pitch[w]) + model['grade_offset']
        for name, ped in (('physical_dynamic', pe[w]), ('physical_static', x['pedal'][w])):
          acc = k[g] * table.engine_torque(tab, ped, n) * ratio - res
          errs[name].append(v0 + np.sum(acc[:-1]) * 0.02 - v_true)
        if g in (4, 5, 6):
          lag = 20
          acc = []
          for i in range(s, s + H - 1):
            r = codex.acceleration(g, float(d['v_wheel'][i]), float(pitch[i]), float(np.mean(pc[max(i - lag - 25, a) : i - lag + 25])))
            acc.append(np.nan if r is None else r)
          acc = np.array(acc)
          if np.isfinite(acc).all():
            errs['codex_map'].append(v0 + np.sum(acc) * 0.02 - v_true)
        errs['constant_accel'].append(v0 + x['a'][s] * 5.0 - v_true if np.isfinite(x['a'][s]) else np.nan)
  out = dict(windows=starts, routes=len(routes))
  for name, e in errs.items():
    e = np.array([z for z in e if np.isfinite(z)])
    out[name] = (
      dict(
        n=int(len(e)),
        rms_mps=round(float(np.sqrt(np.mean(e**2))), 3),
        bias_mps=round(float(np.mean(e)), 3),
        p95_mps=round(float(np.percentile(np.abs(e), 95)), 3),
      )
      if len(e)
      else None
    )
  (load.ROOT / 'analysis' / 'transient-validation.json').write_text(json.dumps(out, indent=1))
  print(json.dumps(out, indent=1))


if __name__ == '__main__':
  main()
