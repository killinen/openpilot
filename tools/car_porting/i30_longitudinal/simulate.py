"""Focused actual-MPC/old-PID comparison with an idealized pedal-only plant.

This is not process replay or evidence of real-world safety. It isolates the
initial-state gap experiment with perfect lead observations and known coast
acceleration; no map inversion, model vision, driver intervention or road slope
estimation is simulated. Both variants use the unchanged existing pedal map.
"""

import argparse
import csv
import json
from collections import deque
from types import SimpleNamespace as NS
from pathlib import Path

import numpy as np

from cereal import car
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, CONTROL_N_T_IDX
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS, A_CHANGE_COST, DANGER_ZONE_COST, J_EGO_COST
from openpilot.tools.car_porting.i30_longitudinal.lookup import coast_state_candidate
from openpilot.tools.car_porting.i30_longitudinal.coast import EarlyCoast


def run(scenario, coast, candidate, early_coast=False):
  cp = car.CarParams.new_message()
  cp.carName = 'i30'
  cp.enableGasInterceptor = True
  cp.stoppingControl = True
  cp.longitudinalTuning.kpBP = [0.0, 15.0, 30.0]
  cp.longitudinalTuning.kiBP = [0.0, 15.0, 30.0]
  cp.longitudinalTuning.kpV = [0.1, 0.2, 0.3]
  cp.longitudinalTuning.kiV = [0.02, 0.025, 0.03]
  cp.longitudinalTuning.deadzoneBP = [0.0]
  cp.longitudinalTuning.deadzoneV = [0.0]
  toggles = NS(longitudinalActuatorDelay=0.35, vEgoStopping=1.0, vEgoStarting=1.0, stopAccel=0.0, stoppingDecelRate=0.8, startAccel=0.0)
  pid = LongControl(cp)
  mpc = LongitudinalMpc()
  # Match production FrogPilot weights, not the MPC constructor's unit defaults.
  mpc.set_weights(A_CHANGE_COST, DANGER_ZONE_COST, J_EGO_COST)
  lift = EarlyCoast()
  speed = planned_speed = lead_speed = 20.0
  acceleration = planned_accel = position = 0.0
  lead_position = 55.0
  pedal = -coast / 2.5
  pid.pid.i = 1.6 * pedal  # Start at equilibrium, rather than a startup transient.
  delay = deque([pedal] * 35)  # 0.35 s actuator lag, updated at 100 Hz.
  coast_duration = 0.0
  rows = []
  solver_failures = 0
  changes = 0
  for frame in range(1200):
    t = frame * 0.05
    lead_active = scenario in ('lead_slow', 'lead_brake', 'cut_in', 'approach')
    if scenario == 'approach' and frame == 0:
      lead_position, lead_speed = 110.0, 16.0
    lead_accel = 0.0
    if scenario == 'lead_slow':
      lead_accel = -0.3 if 10 <= t < 20 else 0.5 if 25 <= t < 31 else 0.0
    elif scenario == 'lead_brake':
      lead_accel = -0.8 if 10 <= t < 20 else 0.8 if 25 <= t < 35 else 0.0
    elif scenario == 'cut_in':
      lead_active = t >= 10
      if frame == 200:
        lead_position, lead_speed = position + 30.0, 16.0
    cruise = 15.0 if scenario == 'cruise_drop' and 10 <= t < 30 else 20.0
    radar = NS(
      leadOne=NS(status=lead_active, dRel=lead_position - position, vLead=lead_speed, aLeadK=lead_accel, aLeadTau=1.1, modelProb=1.0), leadTwo=NS(status=False)
    )
    # The production planner's two-second speed filter and integrated a_desired.
    planned_speed += 0.05 / 2.05 * (speed - planned_speed)
    if planned_speed > cruise:
      planned_speed, planned_accel = cruise, min(planned_accel, 0.0)
    if candidate:
      updated = coast_state_candidate(planned_speed, planned_accel, speed, coast, coast_duration, True, cruise)
      changes += updated != (planned_speed, planned_accel)
      planned_speed, planned_accel = updated
    mpc.set_cur_state(planned_speed, planned_accel)
    empty = np.zeros(len(T_IDXS))
    mpc.update(radar, cruise, empty.copy(), empty.copy(), empty.copy(), empty.copy(), 2.3)
    solver_failures += mpc.solution_status != 0
    speeds = np.interp(CONTROL_N_T_IDX, T_IDXS, mpc.v_solution)
    accels = np.interp(CONTROL_N_T_IDX, T_IDXS, mpc.a_solution)
    capped = np.minimum(speeds, cruise)
    if np.any(capped != speeds):
      speeds, accels = capped, np.gradient(capped, CONTROL_N_T_IDX)
    blocked = early_coast and lift.update(lead_active, speed, lead_position - position, lead_speed - speed, lead_accel, coast, 2.3, 0.35)
    plan = NS(speeds=speeds, accels=accels, allowThrottle=not blocked)
    a_previous = planned_accel
    planned_accel = float(np.interp(0.05, CONTROL_N_T_IDX, accels))
    planned_speed += 0.05 * (planned_accel + a_previous) / 2
    if planned_speed > cruise:
      planned_speed, planned_accel = cruise, min(planned_accel, 0.0)
    for substep in range(5):
      cs = NS(vEgo=speed, aEgo=acceleration, brakePressed=False, cruiseState=NS(standstill=False))
      request = pid.update_old_long(True, cs, plan, [-3.5, 2.0], substep * 0.01, toggles)
      pedal = float(np.clip(request / 1.6, 0.0, 1.0))
      delay.append(pedal)
      acceleration = coast + 2.5 * delay.popleft()
      speed = max(0.0, speed + acceleration * 0.01)
      position += speed * 0.01
      lead_speed = max(0.0, lead_speed + lead_accel * 0.01)
      lead_position += lead_speed * 0.01
      coast_duration = coast_duration + 0.01 if pedal <= 0.001 else 0.0
    rows.append(
      dict(
        t=t,
        speed=speed,
        lead_speed=lead_speed,
        gap=lead_position - position if lead_active else float('nan'),
        pedal=pedal,
        request=request,
        accel=acceleration,
        planned_speed=planned_speed,
        planned_accel=planned_accel,
      )
    )
  gaps = [row['gap'] for row in rows if np.isfinite(row['gap'])]
  return dict(
    scenario=scenario,
    coast=coast,
    candidate=candidate,
    early_coast=early_coast,
    minimum_gap=min(gaps) if gaps else None,
    state_correction_frames=changes,
    solver_failures=solver_failures,
    final_speed=speed,
    final_lead_speed=lead_speed,
    maximum_speed_state_gap=max(row['speed'] - row['planned_speed'] for row in rows),
  ), rows


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--output', type=Path, default=Path('/audit/candidate'))
  args = parser.parse_args()
  args.output.mkdir(parents=True, exist_ok=True)
  results = []
  for scenario in ('steady', 'lead_slow', 'lead_brake', 'cut_in', 'cruise_drop', 'approach'):
    for coast in (-0.35, -0.15):
      for candidate, early_coast in ((False, False), (True, False), (False, True)):
        result, rows = run(scenario, coast, candidate, early_coast)
        results.append(result)
        with (args.output / f'sim-{scenario}-{coast}-{candidate}-{early_coast}.csv').open('w') as f:
          writer = csv.DictWriter(f, fieldnames=list(rows[0]))
          writer.writeheader()
          writer.writerows(rows)
        print(json.dumps(result), flush=True)
  (args.output / 'simulation.json').write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
  main()
