from types import SimpleNamespace

import pytest
from cereal import car

from openpilot.selfdrive.car.toyota.interface import CarInterface
from openpilot.selfdrive.car.toyota.values import CAR
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY


class FakeVM:
  def calc_curvature(self, angle, speed, roll):
    return angle / 40.0 + roll / 100.0


def inputs(speed=20.0, angle=0.0, yaw=0.0):
  cs = SimpleNamespace(vEgo=speed, steeringAngleDeg=angle, steeringPressed=False)
  params = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)
  llk = SimpleNamespace(angularVelocityCalibrated=SimpleNamespace(valid=True, value=[0.0, 0.0, yaw]))
  return cs, params, llk


def controller(fingerprint=CAR.TOYOTA_PRIUS):
  cp = car.CarParams.new_message(steerLimitTimer=.4)
  cp.carFingerprint = fingerprint
  torque = cp.lateralTuning.init('torque')
  torque.latAccelFactor = 2.0
  torque.useSteeringAngle = True
  ci = CarInterface.__new__(CarInterface)
  ci.CP = cp
  return LatControlTorque(cp, ci, .01)


@pytest.mark.parametrize('speed_kph,scale', [(0, 1), (30, 1), (35, 1.075), (40, 1.15), (45, 1.3), (50, 1.45),
                                            (60, 1.45), (90, 1.45), (130, 1.45), (140, 1.45), (150, 1.45),
                                            (200, 1.45)])
def test_hrr_feedforward_speed_schedule(speed_kph, scale):
  c = controller(CAR.LEXUS_LS600h)
  assert c.torque_feedforward_scale(speed_kph / 3.6) == pytest.approx(scale)


@pytest.mark.parametrize('speed_kph', [0, 45, 50, 60, 70, 80, 90, 110, 130, 135, 140, 160])
def test_feedforward_schedule_preserves_other_cars(speed_kph):
  c = controller()
  assert c.torque_feedforward_scale(speed_kph / 3.6) == 1.0


def test_hrr_uses_raised_integral_gain():
  assert controller(CAR.LEXUS_LS600h).pid.k_i == pytest.approx(0.45)
  assert controller().pid.k_i == pytest.approx(0.3)


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('speed_kph', [35, 40, 45, 50, 60, 70, 80, 90, 110, 120, 130, 140, 160])
def test_hrr_scale_changes_only_kinematic_feedforward(direction, speed_kph):
  speed = speed_kph / 3.6
  cs, params, llk = inputs(speed=speed, yaw=direction * .1 / speed)
  params.roll = .005
  baseline = controller()
  hrr = controller(CAR.LEXUS_LS600h)
  target = direction * .2
  # Same Ki on both so this compares only the feedforward scale.
  hrr.pid._k_i = baseline.pid._k_i
  for c in (baseline, hrr):
    c.pid._k_p = [[0], [.85]]
    c.torque_params.friction = .07
    c.torque_params.latAccelOffset = .02
    c.pid.i = direction * .03
  base_steer, _, base_log = baseline.update(True, cs, FakeVM(), params, False, target / speed**2, False, .33, llk, None, None)
  hrr_steer, _, hrr_log = hrr.update(True, cs, FakeVM(), params, False, target / speed**2, False, .33, llk, None, None)
  kinematic_ff = target - params.roll * ACCELERATION_DUE_TO_GRAVITY - .02
  extra_ff = (hrr.torque_feedforward_scale(speed) - 1) * kinematic_ff
  assert hrr_log.f - base_log.f == pytest.approx(extra_ff, abs=1e-7)
  for field in ['error', 'p', 'i', 'd', 'actualLateralAccel', 'desiredLateralAccel']:
    assert getattr(hrr_log, field) == pytest.approx(getattr(base_log, field))
  assert hrr_steer - base_steer == pytest.approx(-extra_ff / 2.0)
  assert hrr.pid.pos_limit == baseline.pid.pos_limit == 2.0
  assert hrr.pid.neg_limit == baseline.pid.neg_limit == -2.0


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('speed_kph', [40, 60, 90, 130, 160])
def test_hrr_feedforward_preserves_controller_authority(direction, speed_kph):
  speed = speed_kph / 3.6
  cs, params, llk = inputs(speed=speed)
  c = controller(CAR.LEXUS_LS600h)
  steer, _, _ = c.update(True, cs, FakeVM(), params, False, direction * 10 / speed**2, False, .33, llk, None, None)
  assert steer == -direction
