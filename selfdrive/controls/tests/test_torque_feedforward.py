import math
from types import SimpleNamespace

import numpy as np

import pytest
from cereal import car

from openpilot.selfdrive.car.toyota.interface import CarInterface
from openpilot.selfdrive.car.toyota.values import CAR
from openpilot.selfdrive.controls.lib.latcontrol_torque import DEADBAND_COMP_WIDTH, LatControlTorque
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
                                            (60, 1.45), (70, 1.45), (80, 1.35), (90, 1.25), (130, 1.25),
                                            (200, 1.25)])
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
  # Same Ki and no deadband compensation, so this compares only the feedforward scale.
  hrr.pid._k_i = baseline.pid._k_i
  hrr.torque_deadband_comp = lambda v_ego: 0.0
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
  assert steer == pytest.approx(-direction, abs=1e-3)
  assert abs(steer) <= 1.0


@pytest.mark.parametrize('speed_kph,torque', [(0, 0), (30, 0), (35, 0), (42.5, .05), (50, .1), (60, .1), (70, .1),
                                              (80, .05), (90, 0), (95, 0), (130, 0)])
def test_hrr_deadband_comp_speed_schedule(speed_kph, torque):
  assert controller(CAR.LEXUS_LS600h).torque_deadband_comp(speed_kph / 3.6) == pytest.approx(torque)


@pytest.mark.parametrize('speed_kph', [0, 30, 50, 70, 90, 130])
def test_deadband_comp_off_for_other_cars(speed_kph):
  assert controller().torque_deadband_comp(speed_kph / 3.6) == 0.0


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('speed_kph', [45, 60, 80])
def test_hrr_deadband_comp_adds_shaped_torque(direction, speed_kph):
  speed = speed_kph / 3.6
  cs, params, llk = inputs(speed=speed)
  c = controller(CAR.LEXUS_LS600h)
  steer, _, _ = c.update(True, cs, FakeVM(), params, False, direction * .2 / speed**2, False, .33, llk, None, None)
  d0 = c.torque_deadband_comp(speed)
  out = c.pid.control
  expected = out / 2.0 + d0 * math.tanh(out / DEADBAND_COMP_WIDTH)
  assert -steer == pytest.approx(expected)
  assert np.sign(steer) == -direction
  assert c.pid.pos_limit == pytest.approx((1 - d0) * 2.0)
  assert c.pid.neg_limit == pytest.approx(-(1 - d0) * 2.0)


def test_hrr_deadband_comp_leaves_motorway_unchanged():
  speed = 120 / 3.6
  cs, params, llk = inputs(speed=speed)
  hrr = controller(CAR.LEXUS_LS600h)
  steer, _, _ = hrr.update(True, cs, FakeVM(), params, False, .3 / speed**2, False, .33, llk, None, None)
  assert -steer == pytest.approx(hrr.pid.control / 2.0)
  assert hrr.pid.pos_limit == 2.0


def test_hrr_deadband_comp_limits_follow_speed():
  c = controller(CAR.LEXUS_LS600h)
  for speed_kph, limit in ((60, 1.8), (130, 2.0), (60, 1.8)):
    cs, params, llk = inputs(speed=speed_kph / 3.6)
    c.update(True, cs, FakeVM(), params, False, 0.0, False, .33, llk, None, None)
    assert c.pid.pos_limit == pytest.approx(limit)


def test_hrr_live_params_keep_deadband_limits():
  c = controller(CAR.LEXUS_LS600h)
  cs, params, llk = inputs(speed=60 / 3.6)
  c.update(True, cs, FakeVM(), params, False, 0.0, False, .33, llk, None, None)
  c.update_live_torque_params(1.2, .01, .1)
  assert c.pid.pos_limit == pytest.approx(.9 * 1.2)
  assert c.pid.neg_limit == pytest.approx(-.9 * 1.2)
  steer, _, _ = c.update(True, cs, FakeVM(), params, False, .2 / (60 / 3.6)**2, False, .33, llk, None, None)
  out = c.pid.control
  assert -steer == pytest.approx(out / 1.2 + .1 * math.tanh(out / DEADBAND_COMP_WIDTH))


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('freeze', [False, True])
def test_hrr_lowered_limit_clamps_integral(direction, freeze):
  c = controller(CAR.LEXUS_LS600h)
  c.update_live_torque_params(1.63, 0.0, .07)
  c.pid.i = direction * 1.60  # inside the motorway limit of 1.63
  cs, params, llk = inputs(speed=60 / 3.6)
  cs.steeringPressed = freeze
  c.update(True, cs, FakeVM(), params, False, 0.0, False, .33, llk, None, None)
  assert c.pid.pos_limit == pytest.approx(.9 * 1.63)
  assert abs(c.pid.i) <= .9 * 1.63 + 1e-9
  assert np.sign(c.pid.i) == direction


def test_hrr_reengage_starts_without_old_integral():
  c = controller(CAR.LEXUS_LS600h)
  c.pid.i = .5
  c.reset()  # what controlsd does every frame lateral is inactive
  assert c.pid.i == 0.0


def test_other_cars_keep_integral_on_disengage():
  c = controller()
  c.pid.i = .5
  c.reset()
  assert c.pid.i == .5
