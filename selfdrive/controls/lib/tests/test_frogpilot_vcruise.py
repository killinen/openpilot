import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def cruise(monkeypatch):
  # Isolate speed arbitration from persistent settings, map access and training.
  def noop(*args):
    pass

  dependencies = {
    "openpilot.frogpilot.common.frogpilot_variables": {"CRUISING_SPEED": 5, "PLANNER_TIME": 10},
    "openpilot.frogpilot.controls.lib.curve_speed_controller": {"CurveSpeedController": noop},
    "openpilot.frogpilot.controls.lib.speed_limit_controller": {"SpeedLimitController": noop},
  }
  for name, attributes in dependencies.items():
    module = ModuleType(name)
    module.__dict__.update(attributes)
    monkeypatch.setitem(sys.modules, name, module)

  path = Path(__file__).resolve().parents[4] / "frogpilot/controls/lib/frogpilot_vcruise.py"
  spec = importlib.util.spec_from_file_location("frogpilot_vcruise_under_test", path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)

  planner = SimpleNamespace(cem=SimpleNamespace(stop_light_detected=False), model_stopped=False,
                            road_curvature_detected=True, model_length=100)
  controller = module.FrogPilotVCruise(planner)
  controller.csc = SimpleNamespace(target=20., target_set=True, update_target=noop, log_data=noop)
  controller.slc = SimpleNamespace(target=15., offset=0., overridden_speed=0., update_limits=noop, update_override=noop)
  sm = {
    "controlsState": SimpleNamespace(enabled=True, vCruiseCluster=108.),
    "carState": SimpleNamespace(vEgoCluster=20., gasPressed=False, standstill=False),
    "frogpilotCarState": SimpleNamespace(accelPressed=False, dashboardSpeedLimit=0.),
    "frogpilotNavigation": SimpleNamespace(navigationSpeedLimit=0.),
    "carControl": SimpleNamespace(longActive=True),
  }
  toggles = SimpleNamespace(force_stops=False, curve_speed_controller=True, speed_limit_controller=True, show_speed_limits=True)
  return controller, sm, toggles


@pytest.mark.parametrize(("csc", "set_speed", "limit", "expected", "controlling"), [
  (25., 30., 15., 15., False),  # Map limit below the curve target.
  (25., 15., 30., 15., False),  # Set speed below the curve target.
  (10., 30., 15., 10., True),
  (15., 30., 15., 15., False),  # A tied curve does not lower the cap.
  (15., 15., 30., 15., False),
  (5., 30., 15., 5., True),    # Curve floor must not fall back to cruise.
  (10., 30., 5., 5., False),   # Map limit at the same supported boundary.
  (10., 30., 0., 10., True),   # Missing map limit.
])
def test_lowest_target_controls_speed(cruise, csc, set_speed, limit, expected, controlling):
  controller, sm, toggles = cruise
  controller.csc.target = csc
  controller.slc.target = limit
  result = controller.update(None, 0, True, set_speed, 20., sm, toggles)
  assert result == pytest.approx(expected)
  assert controller.csc_controlling_speed is controlling


def test_curve_floor_remains_active_across_updates(cruise):
  controller, sm, toggles = cruise
  for target in (5.1, 5., 5., 5.1):
    controller.csc.target = target
    assert controller.update(None, 0, True, 30., 20., sm, toggles) == pytest.approx(target)
    assert controller.csc_controlling_speed


def test_curve_above_set_speed_with_slc_disabled(cruise):
  controller, sm, toggles = cruise
  toggles.speed_limit_controller = False
  assert controller.update(None, 0, True, 15., 20., sm, toggles) == pytest.approx(15.)
  assert not controller.csc_controlling_speed


def test_offset_cluster_correction_and_driver_override_preserve_caps(cruise):
  controller, sm, toggles = cruise
  controller.csc.target = 25.
  controller.slc.offset = 2.
  sm["carState"].vEgoCluster = 21.
  assert controller.update(None, 0, True, 30., 20., sm, toggles) == pytest.approx(16.)
  assert not controller.csc_controlling_speed

  controller.slc.overridden_speed = 40.
  assert controller.update(None, 0, True, 24., 20., sm, toggles) == pytest.approx(24.)
  assert not controller.csc_controlling_speed


def test_curve_status_clears_when_curve_ends(cruise):
  controller, sm, toggles = cruise
  controller.csc.target = 10.
  assert controller.update(None, 0, True, 30., 20., sm, toggles) == pytest.approx(10.)
  assert controller.csc_controlling_speed
  controller.frogpilot_planner.road_curvature_detected = False
  assert controller.update(None, 0, True, 30., 20., sm, toggles) == pytest.approx(15.)
  assert not controller.csc_controlling_speed


def test_forced_stop_takes_precedence_over_curve_status(cruise):
  controller, sm, toggles = cruise
  controller.frogpilot_planner.cem.stop_light_detected = True
  controller.frogpilot_planner.model_stopped = True
  controller.csc.target = 10.
  controller.force_stop_timer = 1.
  controller.tracked_model_length = 20.
  toggles.force_stops = True
  assert controller.update(None, 0, True, 30., 20., sm, toggles) < 5.
  assert controller.forcing_stop
  assert not controller.csc_controlling_speed
