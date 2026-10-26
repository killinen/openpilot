"""TRQI headroom must not rescale ordinary torque or relax driver takeover."""
from types import SimpleNamespace

import pytest
from cereal import car

from openpilot.selfdrive.car.i30.interface import CarInterface
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.car.i30.tests.test_steering_mode import controller_modules, new_controller


def state(output_torque=0.0):
  return SimpleNamespace(out=car.CarState.new_message(steeringTorque=0.0), steering_torque_out=output_torque)


def command(steer):
  cc = car.CarControl.new_message(latActive=True)
  cc.actuators.steer = steer
  return cc.as_reader()


def demand(messages):
  payload = next(m[2] for m in messages if m[0] == 0x232)
  raw = int.from_bytes(payload[:2], 'little') & 0xFFF
  return raw - 0x1000 if raw & 0x800 else raw


@pytest.mark.parametrize('sign', [-1, 1])
def test_reaches_twelve_nm_without_changing_slew(tmp_path, monkeypatch, sign):
  _, values, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller, _ = new_controller(module)
  cc = command(sign*1.2)
  outputs = []
  for _ in range(205):
    applied, messages = controller.update(cc, state(), 0, SimpleNamespace())
    outputs.append(demand(messages))
  assert outputs[0] == sign*6
  assert max(abs(b-a) for a, b in zip(outputs, outputs[1:])) <= 6
  assert outputs[-1] == sign*1200
  assert applied.steer == pytest.approx(sign*1.2)
  applied, messages = controller.update(command(sign*1.5), state(), 0, SimpleNamespace())
  assert demand(messages) == sign*1200
  assert values.TrqiSteerLimitParams.STEER_MAX == 1000


def test_normal_range_torque_scale_is_preserved(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller, _ = new_controller(module)
  for _ in range(60):
    applied, messages = controller.update(command(.3), state(), 0, SimpleNamespace())
  assert demand(messages) == 300  # 0.3 still commands 3 Nm, not 3.6 Nm.
  assert applied.steer == pytest.approx(.3)


def test_measured_output_guard_allows_unwind_at_twelve_nm(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller, _ = new_controller(module)
  controller.last_trqi_tq = 1100
  _, messages = controller.update(command(1.2), state(12.1), 0, SimpleNamespace())
  assert demand(messages) == 1100
  _, messages = controller.update(command(.5), state(12.1), 0, SimpleNamespace())
  assert demand(messages) == 1094
  _, messages = controller.update(command(1.2), state(11.9), 0, SimpleNamespace())
  assert demand(messages) == 1100


def test_pid_headroom_extends_without_changing_conversion(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  _, cp = new_controller(module)
  cp.steerLimitTimer = .4
  tp = cp.lateralTuning.init('torque')
  tp.latAccelFactor = 2.22
  ci = object.__new__(CarInterface)
  control = LatControlTorque(cp, ci, .01)
  assert control.steer_max == pytest.approx(1.2)
  assert control.pid.pos_limit == pytest.approx(2.664)
  assert control.pid.neg_limit == pytest.approx(-2.664)
  assert ci.torque_from_lateral_accel_linear(.666, tp) == pytest.approx(.3)
  cp.steeringMode = 'legacy_ssc'
  assert CarInterface.get_steer_max(cp) == 1.0
  assert CarInterfaceBase.get_steer_max(cp) == 1.0
