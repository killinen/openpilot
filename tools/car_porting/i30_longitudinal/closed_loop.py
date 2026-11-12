"""Closed-loop i30 old-long simulation: real planner + real old-long controller + fitted plant.

Run in the repository Docker environment after building the MPC and msgq (see README).
The plant is the fitted engine/vehicle model with torque dynamics, but its parameters are
perturbed from the controller's calibration (torque gain, drag/coast offset, pitch bias) so
the map is not validated against itself. Lead observations are perfect; vision noise, lead
detection latency and driver behaviour beyond scripted events are not modelled.
"""
import argparse
import json
from dataclasses import dataclass, field
from math import sin
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from cereal import car
from openpilot.selfdrive.car.i30.longitudinal_map import I30LongitudinalMap
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import A_CHANGE_COST, DANGER_ZONE_COST, J_EGO_COST, ACCEL_MIN, CRUISE_MIN_ACCEL
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, get_max_accel
from openpilot.selfdrive.modeld.constants import ModelConstants

DT = 0.01
G = 9.81


@dataclass
class Plant:
  cal: dict
  gear: int = 5
  torque_gain: float = 1.0      # true engine torque / calibrated
  resist_offset: float = 0.0    # extra true resistance, m/s^2 (positive = stronger coast)
  v: float = 20.0
  torque: float = 0.0
  pedal_hist: list = field(default_factory=list)

  def __post_init__(self):
    c = self.cal
    self.pb, self.rb = np.array(c['pedal_bp']), np.array(c['rpm_bp'])
    self.tab = np.array(c['torque'])
    self.k = {int(g): v['k'] for g, v in c['gears'].items()}
    self.ratio = {int(g): v['ratio'] for g, v in c['gears'].items()}
    dyn = c['dynamics']
    self.dead = int(round((dyn['dead_time'] + dyn['torque_to_accel_delay']) / DT))
    self.au, self.ad = DT / (dyn['tau_up'] + DT), DT / (dyn['tau_down'] + DT)

  def rpm(self):
    return max(self.ratio[self.gear] * self.v, 800.0)

  def engine(self, ecu_pedal, rpm):
    rows = np.array([np.interp(ecu_pedal, self.pb, row) for row in self.tab])
    return float(np.interp(rpm, self.rb, rows)) * self.torque_gain if ecu_pedal > 0.5 else float(np.interp(rpm, self.rb, rows))

  def step(self, ecu_pedal, grade, clutch=False, brake_decel=0.0):
    self.pedal_hist.append(ecu_pedal)
    delayed = self.pedal_hist[-self.dead - 1] if len(self.pedal_hist) > self.dead else 0.0
    target = self.engine(delayed, self.rpm())
    self.torque += (self.au if target > self.torque else self.ad) * (target - self.torque)
    drive = 0.0 if clutch else self.k[self.gear] * self.torque * self.rpm() / max(self.v, 0.5)
    c = self.cal
    a = drive - c['drag']['c0'] - c['drag']['c2'] * self.v ** 2 - (c['grade']['gain'] * G * sin(grade) + c['grade']['offset']) - self.resist_offset
    a -= brake_decel
    self.v = max(self.v + a * DT, 0.0)
    return a


def scenario_events(name):
  """Returns a function t -> dict of lead/cruise/grade/driver inputs."""
  def f(t, s):
    e = dict(lead=False, lead_dv=0.0, lead_a=0.0, cruise=22.0, grade=0.0, gas=0.0, brake=0.0, clutch=False, pitch_bias=0.0, cut_in=None, lead_reset=None)
    if name == 'steady':
      pass
    elif name == 'cruise_drop':
      e['cruise'] = 16.0 if 10 <= t < 35 else 22.0
    elif name == 'speed_limit_drop':  # effective FrogPilot target lowered (speed limit / curve)
      e['cruise'] = 14.0 if 10 <= t < 40 else 22.0
    elif name == 'lead_slow':
      e.update(lead=True, lead_a=-0.4 if 10 <= t < 20 else 0.5 if 30 <= t < 38 else 0.0)
    elif name == 'lead_brake':
      e.update(lead=True, lead_a=-1.0 if 10 <= t < 18 else 0.6 if 28 <= t < 40 else 0.0)
    elif name == 'lead_hard_brake':
      e.update(lead=True, lead_a=-2.5 if 10 <= t < 14 else 0.0)
    elif name == 'cut_in':
      e.update(lead=t >= 10, cut_in=(10.0, 25.0, -4.0))  # at t=10 a car appears 25 m ahead, 4 m/s slower
    elif name == 'approach':
      e.update(lead=True)
      if t == 0:
        e['lead_reset'] = (120.0, 15.0)
    elif name == 'lead_loss_change':
      e.update(lead=not (15 <= t < 18), lead_a=0.0)
      if abs(t - 18.0) < 1e-9:
        e['lead_reset'] = (35.0, 17.0)  # different, closer, slower lead after the gap
    elif name == 'uphill':
      e['grade'] = 0.05 if 10 <= t < 40 else 0.0
    elif name == 'downhill':
      e['grade'] = -0.05 if 10 <= t < 40 else 0.0
    elif name == 'downhill_lead':  # coasting cannot slow the car: insufficient authority
      e.update(lead=True, grade=-0.06 if t >= 5 else 0.0, lead_a=-0.3 if 12 <= t < 22 else 0.0)
    elif name == 'driver_gas':
      e['gas'] = 45.0 if 10 <= t < 14 else 0.0
    elif name == 'brake_tap':
      e['brake'] = 1.5 if 10 <= t < 11.5 else 0.0
    elif name == 'shift_down':  # clutch press disengages; driver downshifts 5->4 and re-engages
      e['clutch'] = 10 <= t < 11.0
    elif name == 'pitch_bias':
      e['pitch_bias'] = 0.01  # controller's pitch 0.57 deg off (calibration drift)
    return e
  return f


def build_sm(plant, s, e, lead, plan_cruise, t_follow, lc_state):
  orientation = [0.0, s['pitch'], 0.0]
  cs = NS(vEgo=plant.v, aEgo=s['a_ego'], standstill=plant.v < 0.1, steeringAngleDeg=0.0, engineRpm=plant.rpm())
  lead_one = NS(status=lead['active'], dRel=lead['x'] - s['x'], vLead=lead['v'], aLeadK=lead['a'], aLeadTau=1.5, modelProb=1.0 if lead['active'] else 0.0,
                vRel=lead['v'] - plant.v)
  lead_two = NS(status=False, dRel=0.0, vLead=0.0, aLeadK=0.0, aLeadTau=1.5, modelProb=0.0)
  max_a = get_max_accel(plant.v)
  fp = NS(vCruise=plan_cruise, minAcceleration=ACCEL_MIN if lead['active'] else CRUISE_MIN_ACCEL, maxAcceleration=max_a, cscControllingSpeed=False,
          disableThrottle=False, accelerationJerk=A_CHANGE_COST, dangerJerk=DANGER_ZONE_COST, speedJerk=J_EGO_COST, tFollow=t_follow)
  empty = NS(x=[])
  model = NS(position=empty, velocity=empty, acceleration=empty, orientationRate=NS(z=[0.0] * ModelConstants.IDX_N),
             meta=NS(disengagePredictions=NS(gasPressProbs=[1.0, 1.0])), action=NS(desiredAcceleration=0.0, shouldStop=False))
  return {'carState': cs, 'carControl': NS(orientationNED=orientation), 'radarState': NS(leadOne=lead_one, leadTwo=lead_two),
          'frogpilotPlan': fp, 'modelV2': model, 'liveParameters': NS(angleOffsetDeg=0.0),
          'controlsState': NS(experimentalMode=False, longControlState=lc_state, forceDecel=False, vCruise=plan_cruise * 3.6, personality=1)}


def run(scenario, variant, cal, torque_gain=1.0, resist_offset=0.0, duration=60.0, gear=5, v0=20.0, plant_cal=None, lead_noise=False):
  cp = car.CarParams.new_message()
  cp.carName, cp.enableGasInterceptor, cp.openpilotLongitudinalControl, cp.stoppingControl = 'i30', True, True, True
  cp.longitudinalTuning.kpBP, cp.longitudinalTuning.kpV = [0., 15., 30.], [0.1, 0.2, 0.3]
  cp.longitudinalTuning.kiBP, cp.longitudinalTuning.kiV = [0., 15., 30.], [0.02, 0.025, 0.03]
  cp.longitudinalTuning.deadzoneBP, cp.longitudinalTuning.deadzoneV = [0.], [0.]
  cp.steerRatio, cp.wheelbase = 15.0, 2.65
  toggles = NS(longitudinalActuatorDelay=0.2, vEgoStopping=1.0, vEgoStarting=0.5, stopAccel=0.0, stoppingDecelRate=0.8, startAccel=0.0,
               human_acceleration=False, taco_tune=False, old_long_api=True, max_desired_acceleration=4.0)
  loc = LongControl(cp)
  planner = LongitudinalPlanner(cp, init_v=v0)
  # Controller and planner use exactly the calibration passed in (not whatever is on disk).
  ctrl_map = I30LongitudinalMap(cal)
  loc.pedal_map = ctrl_map if 'map' in variant else None
  planner.coast_map = ctrl_map
  if 'lift' not in variant:
    planner.coast_lift = None
  if 'gap' in variant:
    from openpilot.tools.car_porting.i30_longitudinal.lookup import coast_state_candidate
    orig = planner.mpc.set_cur_state

    def set_cur_state(v, a):
      coast = ctrl_map.coast_accel(plant.v, plant.rpm(), s['pitch'])
      v2, a2 = coast_state_candidate(v, a, plant.v, coast if coast is not None else -0.15, s['zero_pedal_s'], True, plan_cruise)
      planner.v_desired_filter.x, planner.a_desired = v2, a2
      orig(v2, a2)
    planner.mpc.set_cur_state = set_cur_state

  plant_cal = plant_cal or cal
  plant = Plant(plant_cal, gear=gear, torque_gain=torque_gain, resist_offset=resist_offset, v=v0)
  # Equilibrium warm start: the true steady pedal for v0, and for the legacy variant the
  # integral holding that pedal (what the old PID has built up after settling on the road).
  ecu_eq = plant.pb[0]
  for p_try in np.linspace(0, 60, 601):
    plant.torque = plant.engine(p_try, plant.rpm())
    drive = plant.k[plant.gear] * plant.torque * plant.rpm() / v0
    if drive - plant_cal['drag']['c0'] - plant_cal['drag']['c2'] * v0 ** 2 - plant_cal['grade']['offset'] - resist_offset >= 0:
      ecu_eq = p_try
      break
  plant.torque = plant.engine(ecu_eq, plant.rpm())
  plant.pedal_hist = [ecu_eq] * (plant.dead + 1)
  if 'map' not in variant:
    loc.pid.i = 1.6 * ctrl_map.ecu_to_command(ecu_eq)
  ev = scenario_events(scenario)
  # Start in equilibrium cruise at v0 (steady pedal, integral at the legacy road-load value).
  s = dict(x=0.0, a_ego=0.0, pitch=0.0, zero_pedal_s=0.0)
  lead = dict(active=False, x=60.0, v=v0, a=0.0)
  e0 = ev(0.0, s)
  if e0['lead_reset']:
    lead['x'], lead['v'] = e0['lead_reset']
  else:
    lead['x'] = v0 * 1.45 + 6.0 + 4.0
  plan = NS(speeds=[v0] * CONTROL_N, accels=[0.0] * CONTROL_N, allowThrottle=True)
  t_follow, rows, active = 1.45, [], True
  rng, noise = np.random.default_rng(1), dict(d=0.0, v=0.0)
  last_plan_t, ecu_pedal = 0.0, 0.0
  disengaged_until, reengage_count = -1.0, 0
  for frame in range(int(duration / DT)):
    t = round(frame * DT, 2)
    e = ev(t, s)
    if e['lead_reset'] and frame > 0:
      lead['x'], lead['v'] = s['x'] + e['lead_reset'][0], e['lead_reset'][1]
    if e['cut_in'] and abs(t - e['cut_in'][0]) < 1e-9:
      lead['x'], lead['v'] = s['x'] + e['cut_in'][1], plant.v + e['cut_in'][2]
    lead['active'], lead['a'] = e['lead'], e['lead_a']
    s['pitch'] = e['grade'] + e['pitch_bias']
    driver_gas, brake, clutch = e['gas'] > 0, e['brake'] > 0, e['clutch']
    if scenario == 'shift_down' and clutch:
      plant.gear = 4
    # Driver gas override, brake and clutch end longitudinal control (controlsd/card behaviour).
    was_active = active
    active = not (driver_gas or brake or clutch) and t >= disengaged_until
    if brake or clutch:
      disengaged_until = t + 1.0  # driver re-engages a second after release
    if active and not was_active:
      reengage_count += 1
    plan_cruise = e['cruise']
    if frame % 5 == 0:
      seen = dict(lead)
      if lead_noise:
        # Vision-like lead error: AR(1) (1 s correlation) on distance (~4 %) and lead speed (~0.6 m/s), plus noisy lead accel.
        noise['d'] = 0.95 * noise['d'] + 0.31 * rng.normal() * 0.04
        noise['v'] = 0.95 * noise['v'] + 0.31 * rng.normal() * 0.6
        seen['x'] = s['x'] + (lead['x'] - s['x']) * (1 + noise['d'])
        seen['v'] = lead['v'] + noise['v']
        seen['a'] = lead['a'] + rng.normal() * 0.3
      sm = build_sm(plant, s, e, seen, plan_cruise, t_follow, loc.long_control_state)
      planner.update(sm, True, toggles)
      plan = NS(speeds=planner.v_desired_trajectory.tolist(), accels=planner.a_desired_trajectory.tolist(), allowThrottle=planner.allow_throttle)
      last_plan_t = t
    cs = NS(vEgo=plant.v, aEgo=s['a_ego'], brakePressed=brake, engineRpm=plant.rpm(), cruiseState=NS(standstill=False))
    if not active:
      loc.reset_old_long(plant.v)
    accel_cmd = loc.update_old_long(active, cs, plan, [-3.5, 2.0], t - last_plan_t, toggles, pitch=s['pitch'])
    if active:
      command = loc.pedal_command if loc.pedal_map is not None else float(np.clip(accel_cmd / 1.6, 0.0, 1.0))
    else:
      command = 0.0  # exact zero while inactive
    ecu_pedal = e['gas'] if driver_gas else ctrl_map.command_to_ecu(command)
    a = plant.step(ecu_pedal, e['grade'], clutch=clutch, brake_decel=e['brake'])
    s['a_ego'] += 0.1 * (a - s['a_ego'])  # carState aEgo is filtered
    s['x'] += plant.v * DT
    s['zero_pedal_s'] = s['zero_pedal_s'] + DT if ecu_pedal <= 0.0 else 0.0
    lead['v'] = max(lead['v'] + lead['a'] * DT, 0.0)
    lead['x'] += lead['v'] * DT
    if frame % 10 == 0:
      rows.append(dict(t=t, v=plant.v, a=a, pedal=ecu_pedal, command=command, accel_cmd=accel_cmd, active=active, allow_throttle=plan.allowThrottle,
                       gap=lead['x'] - s['x'] if lead['active'] else float('nan'), v_lead=lead['v'], v_pid=loc.v_pid, i=float(loc.pid.i),
                       map=loc.pedal_from_map, plan_v0=plan.speeds[0], lc_state=int(loc.long_control_state), cruise=plan_cruise,
                       solver=planner.mpc.solution_status))
  return summarise(scenario, variant, rows, torque_gain, resist_offset, reengage_count), rows


def summarise(scenario, variant, rows, torque_gain, resist_offset, reengage):
  v, cruise, gap = (np.array([r[k] for r in rows]) for k in ('v', 'cruise', 'gap'))
  pedal, act, t = (np.array([r[k] for r in rows]) for k in ('pedal', 'active', 't'))
  gas_inactive = float(np.max(np.array([r['command'] for r in rows])[~act], initial=0.0))
  no_lead = ~np.isfinite(gap)
  settle = (t > 45) & no_lead & act
  both = act[1:] & act[:-1]
  pedal_d = np.abs(np.diff(pedal))[both]
  out = dict(scenario=scenario, variant=variant, torque_gain=torque_gain, resist_offset=resist_offset,
             min_gap=float(np.nanmin(gap)) if np.isfinite(gap).any() else None,
             max_overspeed=float(np.max((v - cruise)[act & no_lead], initial=0.0)),
             max_underspeed=float(np.max((cruise - v)[(t > 5) & act & no_lead & (np.abs(np.gradient(cruise)) < 1e-9)], initial=0.0)),
             final_speed_error=float(np.mean((v - cruise)[settle])) if settle.any() else None,
             pedal_tv=float(np.sum(pedal_d)) / 100.0, max_command_inactive=gas_inactive, reengagements=reengage,
             solver_failures=int(sum(r['solver'] != 0 for r in rows)), throttle_veto_s=float(np.sum(~np.array([r['allow_throttle'] for r in rows])) * 0.1),
             map_fraction=float(np.mean([r['map'] for r in rows if r['active']])) if act.any() else 0.0)
  return out


SCENARIOS = ['steady', 'cruise_drop', 'speed_limit_drop', 'lead_slow', 'lead_brake', 'lead_hard_brake', 'cut_in', 'approach', 'lead_loss_change',
             'uphill', 'downhill', 'downhill_lead', 'driver_gas', 'brake_tap', 'shift_down', 'pitch_bias']
VARIANTS = ['legacy', 'map', 'map+lift', 'map+lift+gap', 'legacy+lift']
PLANTS = [(1.0, 0.0), (0.85, 0.1), (1.15, -0.1)]


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--calibration', type=Path, default=Path(__file__).parents[3] / 'selfdrive/car/i30/pedal_map.json')
  parser.add_argument('--plant', type=Path, help='plant calibration (default: same as controller)')
  parser.add_argument('--lead-noise', action='store_true')
  parser.add_argument('--lift-fraction', type=float, help='override coast_lift LIFT_FRACTION (release = 0.4x)')
  parser.add_argument('--plants', default='all', choices=['all', 'nominal'])
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--scenarios', nargs='*', default=SCENARIOS)
  parser.add_argument('--variants', nargs='*', default=VARIANTS)
  args = parser.parse_args()
  cal = json.loads(args.calibration.read_text())
  if args.lift_fraction is not None:
    import openpilot.selfdrive.car.i30.coast_lift as coast_lift
    coast_lift.LIFT_FRACTION, coast_lift.RELEASE_FRACTION = args.lift_fraction, 0.4 * args.lift_fraction
  plant_cal = json.loads(args.plant.read_text()) if args.plant else None
  args.output.mkdir(parents=True, exist_ok=True)
  results = []
  for scenario in args.scenarios:
    for gain, offset in (PLANTS if args.plants == 'all' else PLANTS[:1]):
      for variant in args.variants:
        res, rows = run(scenario, variant, cal, gain, offset, plant_cal=plant_cal, lead_noise=args.lead_noise)
        res['lead_noise'] = args.lead_noise
        results.append(res)
        print(json.dumps(res), flush=True)
        if gain == 1.0:
          (args.output / f'cl-{scenario}-{variant}.json').write_text(json.dumps(rows))
  (args.output / 'closed-loop.json').write_text(json.dumps(results, indent=1) + '\n')


if __name__ == '__main__':
  main()
