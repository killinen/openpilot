"""Extract aligned audit channels from local qlogs in the repository Docker environment."""

import argparse
import json
import csv
from pathlib import Path
from collections import Counter, defaultdict
import numpy as np
from openpilot.tools.lib.logreader import LogReader

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--logs', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
root = args.output
root.mkdir(parents=True, exist_ok=True)
counts: Counter[str] = Counter()
errors = []
meta = {}
allrows = []
route_stats = {}


def extract(k, m):
  if k == 'carState':
    return dict(
      v=m.vEgo,
      vraw=m.vEgoRaw,
      a=m.aEgo,
      rpm=m.engineRpm,
      gas=m.gas,
      driver_gas=int(m.gasPressed),
      brake=int(m.brakePressed),
      clutch=int(m.clutchPressed),
      can_valid=int(m.canValid),
      steer=m.steeringAngleDeg,
    )
  if k == 'frogpilotCarState':
    return dict(gear=m.estimatedGear, gear_valid=int(m.estimatedGearValid))
  if k == 'carControl':
    return dict(active=int(m.longActive), requested=m.actuators.accel, pitch=m.orientationNED[1] if len(m.orientationNED) == 3 else float('nan'))
  if k == 'carOutput':
    return dict(pedal=m.actuatorsOutput.gas, output=m.actuatorsOutput.accel)
  if k == 'controlsState':
    return dict(
      vpid=m.vPid, p=m.upAccelCmd, i=m.uiAccelCmd, f=m.ufAccelCmd, experimental=int(m.experimentalMode), cruise=m.vCruise, force_decel=int(m.forceDecel)
    )
  if k == 'longitudinalPlan':
    return dict(
      plan_a=m.aTarget,
      plan_a0=m.accels[0] if len(m.accels) else float('nan'),
      plan_v0=m.speeds[0] if len(m.speeds) else float('nan'),
      source=str(m.longitudinalPlanSource),
      allow_throttle=int(m.allowThrottle),
    )
  if k == 'radarState':
    l = m.leadOne
    return dict(lead=int(l.status), drel=l.dRel, vrel=l.vRel, alead=l.aLeadK)
  if k == 'frogpilotPlan':
    return dict(tfollow=m.tFollow, amin=m.minAcceleration, amax=m.maxAcceleration, plan_cruise=m.vCruise)


files = sorted(args.logs.glob('*/qlog*'), key=lambda p: (p.parent.name.rsplit('--', 1)[0], int(p.parent.name.rsplit('--', 1)[1])))
routes = defaultdict(list)
for p in files:
  routes[p.parent.name.rsplit('--', 1)[0]].append(p)
for route, paths in routes.items():
  streams = defaultdict(list)
  for path in paths:
    try:
      for e in LogReader(str(path)):
        k = e.which()
        counts[k] += 1
        if k == 'carParams':
          m = e.carParams
          meta[route] = dict(
            car=m.carName, fingerprint=m.carFingerprint, delay=m.longitudinalActuatorDelay, kp=list(m.longitudinalTuning.kpV), ki=list(m.longitudinalTuning.kiV)
          )
        elif k == 'initData':
          (root / (route + '-init.json')).write_text(
            json.dumps({k: e.initData.to_dict().get(k) for k in ('gitCommit', 'gitBranch', 'version', 'wallTimeNanos')}, indent=2)
          )
        elif k in ('carState', 'frogpilotCarState', 'carControl', 'carOutput', 'controlsState', 'longitudinalPlan', 'radarState', 'frogpilotPlan'):
          v = extract(k, getattr(e, k))
          v['valid_' + k] = int(e.valid)
          streams[k].append((e.logMonoTime / 1e9, v))
    except Exception as ex:
      errors.append((str(path), repr(ex)))
  for k in streams:
    streams[k].sort(key=lambda x: x[0])
  if not streams['carState']:
    continue
  times = {k: np.array([v[0] for v in vs]) for k, vs in streams.items()}
  route_rows = []
  for t, base in streams['carState']:
    row = dict(route=route, t=t, **base)
    for k in ('frogpilotCarState', 'carControl', 'carOutput', 'controlsState', 'longitudinalPlan', 'radarState', 'frogpilotPlan'):
      if k not in times:
        continue
      ix = np.searchsorted(times[k], t, side='right') - 1
      if ix >= 0:
        age = t - times[k][ix]
        if age <= (0.35 if k in ('longitudinalPlan', 'radarState', 'frogpilotPlan') else 0.15):
          row.update(streams[k][ix][1])
    route_rows.append(row)
  allrows.extend(route_rows)
  route_stats[route] = {
    'segments': len(paths),
    'samples': len(route_rows),
    'seconds': round(sum(min(max(b['t'] - a['t'], 0), 0.2) for a, b in zip(route_rows, route_rows[1:], strict=False)), 1),
  }
  print(route, route_stats[route], 'gear samples', dict(Counter(r.get('gear', -1) for r in route_rows)), flush=True)
fields = ['route', 't'] + sorted(set().union(*(r.keys() for r in allrows)) - {'route', 't'})
with (root / 'samples.csv').open('w') as f:
  w = csv.DictWriter(f, fieldnames=fields)
  w.writeheader()
  w.writerows(allrows)
(root / 'extraction.json').write_text(json.dumps(dict(counts=counts, errors=errors, car_params=meta, routes=route_stats), indent=2))
print('DONE', len(allrows), 'rows', len(errors), 'errors', flush=True)

if errors:
  raise RuntimeError(f'{len(errors)} segment extraction errors; see extraction.json')
