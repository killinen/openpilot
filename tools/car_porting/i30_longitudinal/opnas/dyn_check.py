"""Re-check the pedal->torque dynamics on a broad random sample of training drives."""
import json
import random

import numpy as np

import dataset
import dyn
import load
import table
from fit_model import holdout, C3X

routes = sorted({p.parent.name for p in (load.ROOT / 'features' / C3X).glob('*/*.npz') if not holdout(p.parent.name)})
random.Random(1).shuffle(routes)
d = load.load(devices=[C3X], routes=set(routes[:40]))
x = dataset.derive(d)
p, n, T = x['pedal'], d['N'], x['T']
base = np.isfinite(p) & np.isfinite(n) & np.isfinite(T) & (n > 700) & (d['v_wheel'] > 2) & x['stable']
st = base & dyn.settled(d, p)
idx = np.flatnonzero(st)[::5]
tab, _ = table.fit_engine(p[idx], n[idx], T[idx])
ev = np.flatnonzero(base & ~st)[::2]
res = {}
for dead in (0.0, 0.04, 0.08):
  for tu in (0.25, 0.35, 0.5):
    for td in (0.1, 0.2, 0.3):
      pf = dyn.lag_filter(p, dead, tu, td, d)
      k = ev[np.isfinite(pf[ev])]
      res[f'{dead},{tu},{td}'] = float(np.mean(np.abs(table.engine_torque(tab, pf[k], n[k]) - T[k])))
best = min(res, key=lambda k: res[k])
static = float(np.mean(np.abs(table.engine_torque(tab, p[ev], n[ev]) - T[ev])))
out = dict(routes=40, hours=float(len(d['t']) * 0.02 / 3600), best=best, best_mae=res[best], current=res.get('0.04,0.35,0.2'), static_mae=static, grid=res)
(load.ROOT / 'analysis' / 'dynamics-check.json').write_text(json.dumps(out, indent=1))
print(json.dumps({k: v for k, v in out.items() if k != 'grid'}, indent=1))
