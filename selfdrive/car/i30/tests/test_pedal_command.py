"""Interceptor command selection: exact zero when inactive, map pedal only on the old long API."""
import math
from types import SimpleNamespace

import pytest

from cereal import car
from openpilot.selfdrive.car.i30.carcontroller import CarController
from openpilot.selfdrive.car.i30.values import DBC, CAR, I30_LEGACY_PEDAL_ACCEL

DBC_NAME = DBC[CAR.HYUNDAI_I30_GD_2014]['pt']


def car_state():
  out = car.CarState.new_message(vEgo=20.0, steeringAngleDeg=0.0, steeringTorque=0.0, steeringRateDeg=0.0)
  return SimpleNamespace(out=out, steering_torque_out=0.0, i30_angle_offset=0.0, i30_angle_aligned=True,
                         i30_ssc_angle_unwrapped=0.0, i30_ssc_angle_initialized=True)


def sent_gas(controller, long_active, accel, gas, old_long_api=True):
  cc = car.CarControl.new_message(longActive=long_active, enabled=long_active)
  cc.actuators.accel = accel
  cc.actuators.gas = gas
  toggles = SimpleNamespace(old_long_api=old_long_api)
  _, sends = controller.update(cc.as_reader(), car_state(), 0, toggles)
  controller.frame = 0  # always on a GAS_COMMAND frame
  msgs = [m for m in sends if m[0] == 0x200]
  assert msgs, 'GAS_COMMAND must be sent every other frame'
  enable = (msgs[0][2][4] >> 7) & 1  # ENABLE: 39|1@0+ (byte 4, bit 7)
  return controller.gas, enable


@pytest.fixture
def controller():
  cp = car.CarParams.new_message(carName='i30', carFingerprint=CAR.HYUNDAI_I30_GD_2014, openpilotLongitudinalControl=True, enableGasInterceptor=True)
  c = CarController(DBC_NAME, cp, None)
  c.frame = 0
  return c


def test_inactive_is_exact_zero_and_disabled(controller):
  gas, enable = sent_gas(controller, False, 1.0, 0.7)
  assert gas == 0.0 and enable == 0


def test_old_long_uses_mapped_pedal(controller):
  gas, enable = sent_gas(controller, True, 1.0, 0.27)
  assert gas == pytest.approx(0.27) and enable == 1


def test_new_long_api_or_missing_map_pedal_uses_legacy(controller):
  gas, _ = sent_gas(controller, True, 0.8, 0.27, old_long_api=False)
  assert gas == pytest.approx(0.8 / I30_LEGACY_PEDAL_ACCEL)
  gas, _ = sent_gas(controller, True, 0.8, math.nan)
  assert gas == pytest.approx(0.8 / I30_LEGACY_PEDAL_ACCEL)


def test_zero_map_pedal_disables_interceptor(controller):
  gas, enable = sent_gas(controller, True, -0.5, 0.0)
  assert gas == 0.0 and enable == 0
