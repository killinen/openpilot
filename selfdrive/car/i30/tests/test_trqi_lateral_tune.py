"""TRQI lateral tune: friction threshold and setpoint smoothing, without changing other backends or cars."""
from types import SimpleNamespace

import pytest
from cereal import car

from openpilot.selfdrive.car.i30 import interface as i30_interface
from openpilot.selfdrive.car.interfaces import CarInterfaceBase, FRICTION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque

DT = 0.01
LAT_ACCEL_FACTOR = 2.22
FRICTION = 0.14
V_EGO = 15.0
ROAD = SimpleNamespace(roll=0.0, angleOffsetDeg=0.0)


class FakeCI:
  get_torque_feedforward_scale = staticmethod(lambda v_ego: 1.0)

  def __init__(self, threshold, tau):
    self.threshold, self.tau = threshold, tau

  def get_steer_max(self, CP):
    return 1.2

  def get_torque_friction_threshold(self):
    return self.threshold

  def get_torque_setpoint_filter_tau(self):
    return self.tau

  def torque_from_lateral_accel(self):
    return lambda lateral_accel, params: lateral_accel / params.latAccelFactor

  def lateral_accel_from_torque(self):
    return lambda torque, params: torque * params.latAccelFactor


class StraightVM:
  # Measured lateral acceleration stays at zero; only the request moves.
  def calc_curvature(self, angle, v_ego, roll):
    return 0.0


def controller(threshold=FRICTION_THRESHOLD, tau=0.0):
  CP = car.CarParams.new_message(steerLimitTimer=0.4)
  CP.lateralTuning.init("torque")
  CP.lateralTuning.torque.latAccelFactor = LAT_ACCEL_FACTOR
  CP.lateralTuning.torque.friction = FRICTION
  return LatControlTorque(CP, FakeCI(threshold, tau), DT)


def step(lac, lat_accel, active=True):
  CS = SimpleNamespace(vEgo=V_EGO, steeringAngleDeg=0.0, steeringPressed=False, steeringRateDeg=0.0)
  _, _, pid_log = lac.update(active, CS, StraightVM(), ROAD, False, lat_accel / V_EGO ** 2, False, 0.5, None, None, None)
  return pid_log


@pytest.mark.parametrize("trqi,threshold,tau", [(True, 0.6, 0.05), (False, FRICTION_THRESHOLD, 0.0)])
def test_i30_tune_only_applies_to_trqi(monkeypatch, trqi, threshold, tau):
  monkeypatch.setattr(i30_interface, "i30_uses_trqi_steering", lambda: trqi)
  ci = i30_interface.CarInterface.__new__(i30_interface.CarInterface)
  assert ci.get_torque_friction_threshold() == threshold
  assert ci.get_torque_setpoint_filter_tau() == tau


def test_other_cars_keep_defaults():
  assert CarInterfaceBase.get_torque_friction_threshold(None) == FRICTION_THRESHOLD
  assert CarInterfaceBase.get_torque_setpoint_filter_tau(None) == 0.0


def test_zero_tau_passes_setpoint_through():
  assert step(controller(), 1.0).desiredLateralAccel == pytest.approx(1.0)


def test_setpoint_filter_smooths_a_step_then_settles():
  lac = controller(tau=0.05)
  step(lac, 0.0, active=False)
  first = step(lac, 1.0).desiredLateralAccel
  assert first == pytest.approx(DT / (0.05 + DT))
  for _ in range(100):
    last = step(lac, 1.0).desiredLateralAccel
  assert last == pytest.approx(1.0, abs=1e-3)


def test_engaging_does_not_step_from_stale_filter_state():
  lac = controller(tau=0.05)
  step(lac, 1.5, active=False)
  assert step(lac, 1.5).desiredLateralAccel == pytest.approx(1.5)


def test_doubling_friction_and_threshold_only_adds_torque_above_old_threshold():
  def friction_term(threshold, friction, error):
    lac = controller(threshold=threshold)
    lac.torque_params.friction = friction
    pid_log = step(lac, error)
    return pid_log.f - error

  assert friction_term(0.6, 0.14, 0.2) == pytest.approx(friction_term(0.3, 0.07, 0.2))
  assert friction_term(0.3, 0.07, 0.5) == pytest.approx(0.07 * LAT_ACCEL_FACTOR)
  assert friction_term(0.6, 0.14, 0.5) == pytest.approx(0.14 * LAT_ACCEL_FACTOR * 0.5 / 0.6)
