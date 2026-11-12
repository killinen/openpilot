"""Open-loop controller replay on logged engaged C3X drives (real inputs, not counterfactual outcomes).

Feeds logged carState, longitudinalPlan (speeds/accels/allowThrottle) and controller pitch into
LongControl.update_old_long at 100 Hz. With the legacy conversion the replayed command should
track the logged interceptor command (harness check; logged drives ran older revisions). With the
map it reports where the map is active, why it falls back, and how the pedal moves on real inputs.
The car's response is the logged one, so this says nothing about closed-loop outcomes.
"""
import argparse
import gzip
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from cereal import car
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.longcontrol import LongControl


def channels(path):
  for line in gzip.open(path):
    item = json.loads(line)
    if item.get('type') == 'segment':
      return item['channels']
  return None


def cp():
  c = car.CarParams.new_message(carName='i30', enableGasInterceptor=True, stoppingControl=True, openpilotLongitudinalControl=True)
  c.longitudinalTuning.kpBP, c.longitudinalTuning.kpV = [0., 15., 30.], [0.1, 0.2, 0.3]
  c.longitudinalTuning.kiBP, c.longitudinalTuning.kiV = [0., 15., 30.], [0.02, 0.025, 0.03]
  c.longitudinalTuning.deadzoneBP, c.longitudinalTuning.deadzoneV = [0.], [0.]
  return c


def replay(path, state):
  """Replay one segment; `state` carries controllers across consecutive segments of a drive."""
  ch = channels(path)
  if not ch or not ch['carState']['rows'] or not ch['longitudinalPlan']['rows']:
    return None
  f = {k: {name: i for i, name in enumerate(ch[k]['fields'])} for k in ch}
  cs_rows, plan_rows, cc_rows = ch['carState']['rows'], ch['longitudinalPlan']['rows'], ch['carControl']['rows']
  out_rows = ch['carOutput']['rows']
  plan_t = np.array([r[0] for r in plan_rows])
  cc_t = np.array([r[0] for r in cc_rows])
  out_t = np.array([r[0] for r in out_rows]) if out_rows else np.array([])
  toggles = NS(longitudinalActuatorDelay=0.2, vEgoStopping=1.0, vEgoStarting=0.5, stopAccel=0.0, stoppingDecelRate=0.8, startAccel=0.0,
               max_desired_acceleration=4.0)
  if 'legacy' not in state:
    state['legacy'], state['mapped'] = LongControl(cp()), LongControl(cp())
    state['legacy'].pedal_map = None
    state.update(prev_map=0.0, prev_leg=0.0, engaged_s=0.0, last_t=0)
  legacy, mapped = state['legacy'], state['mapped']
  res = dict(n=0, map_active=0, fallback=0, legacy_cmd=[], logged_cmd=[], map_cmd=[], map_step=[], legacy_step=[], vetoed=0)
  prev_map, prev_leg = state['prev_map'], state['prev_leg']
  for r in cs_rows:
    t = r[0]
    if state['last_t'] and t - state['last_t'] > 0.2e9:  # gap between segments: restart
      state['engaged_s'] = 0.0
      for lc in (legacy, mapped):
        lc.reset_old_long(r[f['carState']['vEgo']])
        lc.long_control_state = car.CarControl.Actuators.LongControlState.off
    state['last_t'] = t
    i = np.searchsorted(plan_t, t, side='right') - 1
    j = np.searchsorted(cc_t, t, side='right') - 1
    if i < 0 or j < 0:
      continue
    pr, cr = plan_rows[i], cc_rows[j]
    active = bool(cr[f['carControl']['longActive']])
    speeds, accels = pr[f['longitudinalPlan']['speeds']], pr[f['longitudinalPlan']['accels']]
    if not isinstance(speeds, list) or len(speeds) != CONTROL_N:
      continue
    plan = NS(speeds=speeds, accels=accels, allowThrottle=bool(pr[f['longitudinalPlan']['allowThrottle']]))
    o = cr[f['carControl']['orientationNED']]
    pitch = o[1] if isinstance(o, list) and len(o) == 3 else math.nan
    cs = NS(vEgo=r[f['carState']['vEgo']], aEgo=r[f['carState']['aEgo']], brakePressed=bool(r[f['carState']['brakePressed']]),
            engineRpm=r[f['carState']['engineRpm']], cruiseState=NS(standstill=False))
    t_since = (t - plan_t[i]) / 1e9
    for lc in (legacy, mapped):
      if not active:
        lc.reset_old_long(cs.vEgo)
    a_leg = legacy.update_old_long(active, cs, plan, [-3.5, 2.0], t_since, toggles, pitch=pitch)
    mapped.update_old_long(active, cs, plan, [-3.5, 2.0], t_since, toggles, pitch=pitch)
    state['engaged_s'] = state['engaged_s'] + 0.01 if active else 0.0
    if not active:
      prev_map = prev_leg = 0.0
      continue
    if state['engaged_s'] < 30.0:  # let the legacy integral reach road load before comparing
      prev_map, prev_leg = mapped.pedal_command, float(np.clip(a_leg / 1.6, 0, 1))
      continue
    leg_cmd = float(np.clip(a_leg / 1.6, 0, 1)) if plan.allowThrottle else 0.0
    res['n'] += 1
    res['map_active'] += mapped.pedal_from_map
    res['vetoed'] += not plan.allowThrottle
    res['map_cmd'].append(mapped.pedal_command)
    res['legacy_cmd'].append(leg_cmd)
    res['map_step'].append(abs(mapped.pedal_command - prev_map))
    res['legacy_step'].append(abs(leg_cmd - prev_leg))
    prev_map, prev_leg = mapped.pedal_command, leg_cmd
    k = np.searchsorted(out_t, t, side='right') - 1 if len(out_t) else -1
    res['logged_cmd'].append(out_rows[k][f['carOutput']['actuatorsOutput.gas']] if k >= 0 else math.nan)
  state['prev_map'], state['prev_leg'] = prev_map, prev_leg
  return res


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--raw', type=Path, required=True, help='I30_LONG_DATA/raw/<C3X device>')
  p.add_argument('--max-segments', type=int, default=200)
  p.add_argument('--output', type=Path, required=True)
  a = p.parse_args()
  routes = sorted({fp.parent for fp in a.raw.glob('*/*.jsonl.gz')})
  rng = np.random.default_rng(0)
  routes = [routes[i] for i in rng.permutation(len(routes))]
  tot = dict(n=0, map_active=0, vetoed=0)
  cols = {k: [] for k in ('legacy_cmd', 'logged_cmd', 'map_cmd', 'map_step', 'legacy_step')}
  used = 0
  for route in routes:
    if used >= a.max_segments:
      break
    state = {}
    segs = sorted(route.glob('*.jsonl.gz'), key=lambda q: int(q.name.split('.')[0]))
    for fp in segs:
      r = replay(fp, state)
      if not r:
        state = {}
        continue
      used += 1
      for k in tot:
        tot[k] += r[k]
      for k in cols:
        cols[k] += r[k]
  c = {k: np.array(v, dtype=float) for k, v in cols.items()}
  ok = np.isfinite(c['logged_cmd'])
  out = dict(segments=used, engaged_hours=round(tot['n'] * 0.01 / 3600, 2), map_active_fraction=round(tot['map_active'] / max(tot['n'], 1), 3),
             vetoed_fraction=round(tot['vetoed'] / max(tot['n'], 1), 4),
             legacy_replay_vs_logged=dict(mae=round(float(np.mean(np.abs(c['legacy_cmd'][ok] - c['logged_cmd'][ok]))), 4),
                                          corr=round(float(np.corrcoef(c['legacy_cmd'][ok], c['logged_cmd'][ok])[0, 1]), 3)),
             mean_cmd=dict(logged=round(float(np.nanmean(c['logged_cmd'])), 4), legacy=round(float(c['legacy_cmd'].mean()), 4),
                           map=round(float(c['map_cmd'].mean()), 4)),
             zero_cmd_fraction=dict(legacy=round(float(np.mean(c['legacy_cmd'] == 0)), 3), map=round(float(np.mean(c['map_cmd'] == 0)), 3)),
             step_per_10ms_p99_max=dict(legacy=[round(float(np.percentile(c['legacy_step'], 99)), 4), round(float(c['legacy_step'].max()), 3)],
                                        map=[round(float(np.percentile(c['map_step'], 99)), 4), round(float(c['map_step'].max()), 3)]))
  a.output.write_text(json.dumps(out, indent=1) + '\n')
  print(json.dumps(out, indent=1))


if __name__ == '__main__':
  main()
