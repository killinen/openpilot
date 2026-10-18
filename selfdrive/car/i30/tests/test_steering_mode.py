"""The logged mode must describe the running controller, including default SSC."""

import importlib
import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import pytest

from cereal import car, log
from openpilot.common import params as params_module
from openpilot.tools.teletyped.drive_stats import DriveAnalyzer


def controller_modules(tmp_path, monkeypatch, toggle):
  # Keep the real Params implementation, isolated from the device's settings.
  params = params_module.Params(str(tmp_path / "params"))
  if toggle is not None:
    params.put("TRQISteeringToggle", toggle)
  monkeypatch.setattr(params_module, "Params", lambda: params)
  from openpilot.selfdrive.car.i30 import values
  importlib.reload(values)
  # Load the real controller constructor without requiring the native CAN
  # packer or unrelated vehicle interfaces. Neither participates in labeling.
  packer = ModuleType("opendbc.can.packer")
  packer.CANPacker = lambda _dbc: None
  interfaces = ModuleType("openpilot.selfdrive.car.interfaces")
  interfaces.CarControllerBase = object
  spec = importlib.util.spec_from_file_location("i30_mode_controller_test", Path(__file__).resolve().parents[1] / "carcontroller.py")
  module = importlib.util.module_from_spec(spec)
  with monkeypatch.context() as imports:
    imports.setitem(sys.modules, "opendbc.can.packer", packer)
    imports.setitem(sys.modules, "openpilot.selfdrive.car.interfaces", interfaces)
    spec.loader.exec_module(module)
  return params, values, module


def new_controller(module):
  cp = car.CarParams.new_message(carName="i30", carFingerprint="HYUNDAI I30 GD 2014")
  return module.CarController("", cp, None), cp


@pytest.mark.parametrize("toggle,mode", [
  (None, "legacy_ssc"), (b"0", "legacy_ssc"), (b"1", "trqi_tq_delta"),
])
def test_controller_records_actual_startup_mode_in_serialized_car_params(tmp_path, monkeypatch, toggle, mode):
  _params, _values, module = controller_modules(tmp_path, monkeypatch, toggle)
  controller, cp = new_controller(module)
  assert controller.use_trqi_steering == (mode == "trqi_tq_delta")
  assert cp.steeringMode == mode
  # Use a real logged Event so this verifies the producer/collector wire format.
  event = log.Event.new_message()
  event.init("carParams")
  event.carParams = cp
  with log.Event.from_bytes(event.to_bytes()) as recorded:
    analyzer = DriveAnalyzer("new-route", "DONGLE")
    analyzer.process_segment([recorded])
    result = analyzer.result()
  assert result["steering_mode"] == mode
  assert result["steer_intervention_source"] == ("steering_pressed" if controller.use_trqi_steering else "torque_delta")


@pytest.mark.parametrize("toggle,replacement", [(b"0", b"1"), (b"1", b"0")])
def test_toggle_change_only_changes_logged_mode_after_controller_startup_reloads(tmp_path, monkeypatch, toggle, replacement):
  params, values, module = controller_modules(tmp_path, monkeypatch, toggle)
  controller, cp = new_controller(module)
  original_mode = cp.steeringMode
  params.put("TRQISteeringToggle", replacement)
  assert controller.use_trqi_steering == (toggle == b"1")
  assert cp.steeringMode == original_mode

  # Emulate a later route snapshot containing the newly saved toggle. The
  # collector must still honor the already-running controller's logged mode.
  init = log.Event.new_message()
  init.init("initData")
  entry = init.initData.init("params").init("entries", 1)[0]
  entry.key = "TRQISteeringToggle"
  entry.value = replacement
  recorded_cp = log.Event.new_message()
  recorded_cp.init("carParams")
  recorded_cp.carParams = cp
  analyzer = DriveAnalyzer("before-restart", "DONGLE")
  analyzer.process_segment([init, recorded_cp])
  assert analyzer.result()["steering_mode"] == original_mode

  importlib.reload(values)
  restarted, new_cp = new_controller(module)
  assert restarted.use_trqi_steering == (replacement == b"1")
  assert new_cp.steeringMode != original_mode
  assert cp.steeringMode == original_mode
