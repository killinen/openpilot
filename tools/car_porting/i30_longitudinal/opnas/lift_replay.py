"""Offline observation of the early-lift veto on logged engaged following (not counterfactual replay)."""

import json
import sys
import numpy as np
import os
from pathlib import Path

REPO = os.environ.get('I30_REPO', str(Path(__file__).resolve().parents[4]))
import load

sys.path.insert(0, REPO + '')
# Import the runtime modules directly from the checkout (pure python, no openpilot deps).
import importlib.util  # noqa: F401
import types


def mod(name, path):
  # Host analysis may run an older Python without zip(strict=); the runtime code targets the device.
  m = types.ModuleType(name)
  m.__file__ = path
  exec(compile(open(path).read().replace(', strict=True', '').replace(', strict=False', ''), path, 'exec'), m.__dict__)
  return m


REPO = REPO + '/selfdrive/car/i30/'
lift_mod = mod('coast_lift', REPO + 'coast_lift.py')
map_mod = mod('longitudinal_map', REPO + 'longitudinal_map.py')


def main():
  cols = [
    't',
    'v_wheel',
    'N',
    'pitch',
    'PV_AV_CAN',
    'carControl.longActive',
    'radarState.leadOne.status',
    'radarState.leadOne.dRel',
    'radarState.leadOne.vRel',
    'radarState.leadOne.aLeadK',
    'brake_act',
    'esp_CYL_PRES',
    'carState.gasPressed',
  ]
  d = load.load(devices=['ff528bde139e99de'], cols=cols)
  m = map_mod.I30LongitudinalMap()
  n = len(d['t'])
  veto = np.zeros(n, bool)
  coast = np.full(n, np.nan)
  need = np.full(n, np.nan)
  for a, b in load.blocks(d):
    lift = lift_mod.CoastLiftOff()
    for i in range(a, b, 5):  # planner rate 10 Hz is enough here
      v = d['v_wheel'][i]
      valid = d['radarState.leadOne.status'][i] == 1 and d['carControl.longActive'][i] == 1
      c = m.coast_accel(v, d['N'][i], d['pitch'][i]) if np.isfinite(v) else None
      vl = v + d['radarState.leadOne.vRel'][i]
      r = lift.update(valid, v, d['radarState.leadOne.dRel'][i], vl, d['radarState.leadOne.aLeadK'][i], 1.45, c)
      veto[i : i + 5] = r
      coast[i : i + 5] = c if c is not None else np.nan
      need[i : i + 5] = lift.needed if valid else np.nan
  eng = d['carControl.longActive'] == 1
  follow = eng & (d['radarState.leadOne.status'] == 1)
  pedal = d['PV_AV_CAN']
  brake = (d['brake_act'] == 2) | (d['esp_CYL_PRES'] > 3)
  # Driver brake onsets that end an engaged following period.
  onsets = np.flatnonzero(brake[1:] & ~brake[:-1] & follow[:-1]) + 1
  leads = []
  for i in onsets:
    lo = max(0, i - 500)
    w = veto[lo:i]
    if w.any():
      first = lo + int(np.argmax(w))
      leads.append((i - first) * 0.02)
    else:
      leads.append(0.0)
  res = dict(
    engaged_hours=float(eng.sum() * 0.02 / 3600),
    following_hours=float(follow.sum() * 0.02 / 3600),
    veto_s=float(veto.sum() * 0.02),
    veto_fraction_of_following=float(veto[follow].mean()) if follow.any() else None,
    throttle_removed_s=float((veto & (pedal > 1.0)).sum() * 0.02),
    coast_known_fraction=float(np.isfinite(coast[follow]).mean()) if follow.any() else None,
    coast_accel_p5_p50_p95=np.nanpercentile(coast[follow], [5, 50, 95]).round(3).tolist() if np.isfinite(coast[follow]).any() else None,
    driver_brake_onsets_while_following=int(len(onsets)),
    onsets_preceded_by_veto=int(sum(x > 0 for x in leads)),
    veto_lead_time_s_p50_p90=np.percentile([x for x in leads if x > 0], [50, 90]).round(2).tolist() if any(x > 0 for x in leads) else None,
    pedal_at_onset_minus_1s_median=float(np.nanmedian([pedal[max(i - 50, 0)] for i in onsets])) if len(onsets) else None,
  )
  (load.ROOT / 'analysis' / 'lift-replay.json').write_text(json.dumps(res, indent=1))
  print(json.dumps(res, indent=1))


if __name__ == '__main__':
  main()
