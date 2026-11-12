import pytest
import numpy as np

from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.longitudinal_planner import cap_trajectory_accel
from openpilot.selfdrive.modeld.constants import ModelConstants

T = ModelConstants.T_IDXS[:CONTROL_N]


def trajectory(a):
  a = np.asarray(a, dtype=float)
  v = 20.0 + np.concatenate(([0.0], np.cumsum((a[1:] + a[:-1]) / 2 * np.diff(T))))
  return v, a


def test_positive_excess_removed_and_speed_lowered():
  v, a = trajectory(np.full(CONTROL_N, 1.5))
  v_cap, a_cap = cap_trajectory_accel(v, a, T, 0.6)
  assert np.allclose(a_cap, 0.6)
  assert v_cap[0] == v[0]
  assert np.all(v_cap <= v + 1e-9)
  assert np.allclose(np.diff(v_cap) / np.diff(T), 0.6)


def test_deceleration_never_weakened():
  a = np.linspace(-3.0, 1.2, CONTROL_N)
  v, a = trajectory(a)
  v_cap, a_cap = cap_trajectory_accel(v, a, T, 0.4)
  decel = a < 0.4
  assert np.array_equal(a_cap[decel], a[decel])
  assert np.all(v_cap <= v + 1e-9)


def test_negative_bound_does_not_force_deceleration():
  v, a = trajectory(np.full(CONTROL_N, 0.5))
  v_cap, a_cap = cap_trajectory_accel(v, a, T, -0.8)
  assert np.allclose(a_cap, 0.0)
  assert np.allclose(v_cap, v[0])


def test_unchanged_below_cap():
  v, a = trajectory(np.full(CONTROL_N, -0.5))
  v_cap, a_cap = cap_trajectory_accel(v, a, T, 1.0)
  assert np.array_equal(v_cap, v)
  assert np.array_equal(a_cap, a)


def test_speeds_never_negative():
  v = np.linspace(0.5, 0.0, CONTROL_N)
  a = np.full(CONTROL_N, 2.0)
  v_cap, _ = cap_trajectory_accel(v, a, T, 0.0)
  assert np.all(v_cap >= 0.0)


# --- planner integration (real MPC) ----------------------------------------
from types import SimpleNamespace as NS

from cereal import car


def planner_sm(v_ego, max_accel, lead=None, v_cruise=30.0):
  empty = NS(x=[])
  lead_one = NS(status=lead is not None, dRel=lead[0] if lead else 0.0, vLead=lead[1] if lead else 0.0, aLeadK=0.0, aLeadTau=1.5,
                modelProb=1.0 if lead else 0.0)
  return {
    'carState': NS(vEgo=v_ego, aEgo=0.0, standstill=False, steeringAngleDeg=0.0, engineRpm=105.7 * v_ego),
    'carControl': NS(orientationNED=[0.0, 0.0, 0.0]),
    'radarState': NS(leadOne=lead_one, leadTwo=NS(status=False, dRel=0.0, vLead=0.0, aLeadK=0.0, aLeadTau=1.5, modelProb=0.0)),
    'frogpilotPlan': NS(vCruise=v_cruise, minAcceleration=-3.5 if lead else -1.2, maxAcceleration=max_accel, cscControllingSpeed=False,
                        disableThrottle=False, accelerationJerk=200.0, dangerJerk=100.0, speedJerk=5.0, tFollow=1.45),
    'modelV2': NS(position=empty, velocity=empty, acceleration=empty, orientationRate=NS(z=[0.0] * 33),
                  meta=NS(disengagePredictions=NS(gasPressProbs=[1.0, 1.0])), action=NS(desiredAcceleration=0.0, shouldStop=False)),
    'liveParameters': NS(angleOffsetDeg=0.0),
    'controlsState': NS(experimentalMode=False, longControlState=car.CarControl.Actuators.LongControlState.pid, forceDecel=False,
                        vCruise=v_cruise * 3.6, personality=1),
  }


def run_planner(old_long_api, max_accel, lead=None, car_name='toyota', steps=40):
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  cp = car.CarParams.new_message(carName=car_name, enableGasInterceptor=car_name == 'i30', openpilotLongitudinalControl=True, steerRatio=15.0,
                                 wheelbase=2.65)
  planner = LongitudinalPlanner(cp, init_v=20.0)
  planner.prev_accel_clip = [-3.5, max_accel]
  toggles = NS(taco_tune=False, longitudinalActuatorDelay=0.2, vEgoStopping=1.0, old_long_api=old_long_api)
  for _ in range(steps):
    planner.update(planner_sm(20.0, max_accel, lead), True, toggles)
  return planner


def test_old_long_trajectory_respects_accel_limit_new_api_unchanged():
  capped = run_planner(True, 0.4)
  assert np.max(capped.a_desired_trajectory) <= 0.4 + 1e-6
  uncapped = run_planner(False, 0.4)
  assert np.max(uncapped.a_desired_trajectory) > 0.6  # MPC itself is not limited by the profile
  assert uncapped.output_a_target <= 0.4 + 1e-6  # new API still clips its scalar


def test_old_long_cap_keeps_lead_deceleration():
  lead = (25.0, 12.0)
  capped = run_planner(True, 0.4, lead)
  plain = run_planner(False, 0.4, lead)
  assert np.min(capped.a_desired_trajectory) < -1.0
  assert np.min(capped.a_desired_trajectory) == pytest.approx(np.min(plain.a_desired_trajectory), abs=0.05)


def test_i30_early_lift_vetoes_throttle_before_needed_braking():
  far = run_planner(True, 1.0, (150.0, 19.5), car_name='i30', steps=5)
  assert far.allow_throttle
  closing = run_planner(True, 1.0, (55.0, 16.0), car_name='i30', steps=5)
  assert not closing.allow_throttle
  other_car = run_planner(True, 1.0, (55.0, 16.0), car_name='toyota', steps=5)
  assert other_car.allow_throttle
