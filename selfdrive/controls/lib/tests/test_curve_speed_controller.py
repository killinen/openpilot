import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

DT = 0.05
N = 33


@pytest.fixture
def csc_module(monkeypatch):
  # Load the controller without persistent params or the full openpilot runtime.
  params = SimpleNamespace(get=lambda *args: None, put_float_nonblocking=lambda *args: None, put_nonblocking=lambda *args: None)
  dependencies = {
    "openpilot.common.realtime": {"DT_MDL": DT},
    "openpilot.frogpilot.common.frogpilot_variables": {"CRUISING_SPEED": 5, "DEFAULT_LATERAL_ACCELERATION": 2.0, "PLANNER_TIME": 10, "params": params},
  }
  for name, attributes in dependencies.items():
    module = ModuleType(name)
    module.__dict__.update(attributes)
    monkeypatch.setitem(sys.modules, name, module)

  path = Path(__file__).resolve().parents[4] / "frogpilot/controls/lib/curve_speed_controller.py"
  spec = importlib.util.spec_from_file_location("curve_speed_controller_under_test", path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def path(v_ego, radius=None, curve_start=None):
  """Constant-speed model path, straight until `curve_start` metres, then a constant radius."""
  t = np.linspace(0, 10, N)
  position = v_ego * t
  velocity = np.full(N, v_ego)
  orientation_rate = np.zeros(N)
  if radius is not None:
    orientation_rate[position >= curve_start] = v_ego / radius
  return orientation_rate, velocity, position


def make_controller(module, min_accel=-1.2):
  planner = SimpleNamespace(frogpilot_weather=SimpleNamespace(weather_id=0, reduce_lateral_acceleration=0.),
                            frogpilot_acceleration=SimpleNamespace(min_accel=min_accel))
  return module.CurveSpeedController(SimpleNamespace(frogpilot_planner=planner))


def make_sm(orientation_rate, velocity, position, blinker=False):
  model = SimpleNamespace(orientationRate=SimpleNamespace(z=list(orientation_rate)), velocity=SimpleNamespace(x=list(velocity)),
                          position=SimpleNamespace(x=list(position)))
  return {"modelV2": model, "carState": SimpleNamespace(leftBlinker=blinker, rightBlinker=False)}


TOGGLES = SimpleNamespace(curve_speed_lateral_acceleration=2.0)


def test_straight_road_has_no_limit(csc_module):
  assert csc_module.calculate_curve_speed_limit(*path(25.), 25., 2.0, 1.0) > 60.


def test_curve_speed_in_curve(csc_module):
  # Already in a 100 m radius curve: sqrt(2.0 * 100) m/s.
  limit = csc_module.calculate_curve_speed_limit(*path(25., radius=100., curve_start=0.), 25., 2.0, 1.0)
  assert limit == pytest.approx(200**0.5, rel=1e-3)


def test_braking_envelope_for_distant_curve(csc_module):
  radius, start, v_ego, decel = 100., 150., 25., 1.0
  orientation_rate, velocity, position = path(v_ego, radius, start)
  limit = csc_module.calculate_curve_speed_limit(orientation_rate, velocity, position, v_ego, 2.0, decel)
  first = position[position >= start][0]
  expected = (2.0 * radius + 2 * decel * (first - v_ego * csc_module.REACTION_TIME))**0.5
  assert limit == pytest.approx(expected, rel=1e-3)
  assert 200**0.5 < limit < v_ego


def test_curvature_uses_model_speed(csc_module):
  # The model already slows to 15 m/s in the curve; curvature must come from that speed, not v_ego.
  orientation_rate, velocity, position = path(25., radius=100., curve_start=0.)
  velocity[:] = 15.
  orientation_rate[:] = 15. / 100.
  limit = csc_module.calculate_curve_speed_limit(orientation_rate, velocity, position, 25., 2.0, 1.0)
  assert limit == pytest.approx(200**0.5, rel=1e-3)


def test_invalid_model_output_has_no_limit(csc_module):
  assert csc_module.calculate_curve_speed_limit([], [], [], 20., 2.0, 1.0) == float("inf")


def test_distant_curve_does_not_step_the_target(csc_module):
  # The old controller jumped straight to the curve speed; the target must stay at v_ego until braking is needed.
  controller = make_controller(csc_module)
  v_ego = 25.
  sm = make_sm(*path(v_ego, radius=100., curve_start=240.))  # Envelope ~25.4 m/s, so no braking yet.
  for _ in range(2):
    controller.update_target(v_ego, sm, TOGGLES)
    assert v_ego < controller.target < v_ego + 1.


def test_target_falls_at_bounded_rate_from_v_ego(csc_module):
  controller = make_controller(csc_module)
  v_ego = 25.
  straight, curve = make_sm(*path(v_ego)), make_sm(*path(v_ego, radius=100., curve_start=0.))
  controller.update_target(v_ego, straight, TOGGLES)
  assert controller.target > 60.
  controller.update_target(v_ego, curve, TOGGLES)
  assert controller.target == pytest.approx(v_ego)
  controller.update_target(v_ego, curve, TOGGLES)
  assert controller.target == pytest.approx(v_ego - csc_module.TARGET_FALL_RATE * DT)
  for _ in range(200):
    controller.update_target(v_ego, curve, TOGGLES)
  assert controller.target == pytest.approx(200**0.5, rel=1e-3)


def test_single_frame_spike_is_rejected(csc_module):
  controller = make_controller(csc_module)
  v_ego = 20.
  straight, spike = make_sm(*path(v_ego)), make_sm(*path(v_ego, radius=30., curve_start=0.))
  controller.update_target(v_ego, straight, TOGGLES)
  controller.update_target(v_ego, spike, TOGGLES)
  assert controller.target > v_ego - 0.2
  controller.update_target(v_ego, straight, TOGGLES)
  assert controller.target >= v_ego


def test_release_after_curve_is_gradual(csc_module):
  controller = make_controller(csc_module)
  v_ego = 14.
  curve, straight = make_sm(*path(v_ego, radius=100., curve_start=0.)), make_sm(*path(v_ego))
  controller.update_target(v_ego, curve, TOGGLES)
  for _ in range(100):
    controller.update_target(v_ego, curve, TOGGLES)
  start = controller.target
  controller.update_target(v_ego, straight, TOGGLES)
  assert controller.target == pytest.approx(max(start + csc_module.TARGET_RISE_RATE * DT, v_ego))


def test_target_never_held_below_v_ego_when_path_allows(csc_module):
  # Engaging while pulling out of a junction must not cap acceleration once the road straightens.
  controller = make_controller(csc_module)
  controller.update_target(6., make_sm(*path(6., radius=10., curve_start=0.)), TOGGLES)
  controller.update_target(12., make_sm(*path(12.)), TOGGLES)
  assert controller.target >= 12.


def test_blinker_releases_gradually(csc_module):
  controller = make_controller(csc_module)
  v_ego = 14.
  curve = make_sm(*path(v_ego, radius=60., curve_start=0.))
  for _ in range(100):
    controller.update_target(v_ego, curve, TOGGLES)
  before = controller.target
  controller.update_target(v_ego, make_sm(*path(v_ego, radius=60., curve_start=0.), blinker=True), TOGGLES)
  assert controller.target == pytest.approx(max(before + csc_module.TARGET_RISE_RATE * DT, v_ego))


@pytest.mark.parametrize(("min_accel", "decel"), [(-1.2, 1.0), (-0.6, 0.6), (-0.2, 0.5), (-3.5, 1.0)])
def test_deceleration_follows_profile(csc_module, min_accel, decel):
  controller = make_controller(csc_module, min_accel)
  v_ego, radius, start = 25., 100., 150.
  orientation_rate, velocity, position = path(v_ego, radius, start)
  sm = make_sm(orientation_rate, velocity, position)
  for _ in range(400):
    controller.update_target(v_ego, sm, TOGGLES)
  expected = csc_module.calculate_curve_speed_limit(orientation_rate, velocity, position, v_ego, 2.0, decel)
  assert controller.target == pytest.approx(min(expected, v_ego), rel=1e-3)
