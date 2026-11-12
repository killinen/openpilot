import pytest
from types import SimpleNamespace

from cereal import car

from openpilot.selfdrive.controls.lib.longcontrol import gas_interceptor_integrator_limits, LongControl, LongCtrlState, CONTROL_N_T_IDX


class TestGasInterceptorIntegratorLimits:
  def test_caps_positive_integral_when_overspeed_and_target_flat(self):
    integrator_min, integrator_max = gas_interceptor_integrator_limits(
      v_ego=31.4,
      v_pid=31.1,
      v_target_1sec=31.1,
      p_term=-0.08,
      feedforward=0.0,
    )

    assert integrator_min == pytest.approx(0.0)
    assert integrator_max == pytest.approx(0.08)

  def test_does_not_cap_while_target_is_accelerating(self):
    integrator_min, integrator_max = gas_interceptor_integrator_limits(
      v_ego=31.4,
      v_pid=31.1,
      v_target_1sec=32.0,
      p_term=-0.08,
      feedforward=0.0,
    )

    assert integrator_min == pytest.approx(0.0)
    assert integrator_max is None

  def test_preserves_integral_when_large_target_drop_already_commands_decel(self):
    integrator_min, integrator_max = gas_interceptor_integrator_limits(
      v_ego=31.0,
      v_pid=25.0,
      v_target_1sec=25.0,
      p_term=-1.8,
      feedforward=0.0,
    )

    assert integrator_min == pytest.approx(0.0)
    assert integrator_max == pytest.approx(1.8)


def interceptor_controller(car_name="i30", interceptor=True):
  cp = car.CarParams.new_message()
  cp.carName = car_name
  cp.enableGasInterceptor = interceptor
  cp.stoppingControl = True
  cp.longitudinalTuning.kpBP = [0.0]
  cp.longitudinalTuning.kpV = [0.2]
  cp.longitudinalTuning.kiBP = [0.0]
  cp.longitudinalTuning.kiV = [0.03]
  cp.longitudinalTuning.deadzoneBP = [0.0]
  cp.longitudinalTuning.deadzoneV = [0.0]
  controller = LongControl(cp)
  controller.pid.i = 0.25
  cs = SimpleNamespace(vEgo=15.0, aEgo=0.0, brakePressed=False, cruiseState=SimpleNamespace(standstill=False))
  toggles = SimpleNamespace(longitudinalActuatorDelay=0.35, vEgoStopping=1.0, vEgoStarting=1.0, stopAccel=0.0, stoppingDecelRate=0.8, startAccel=0.5)
  return controller, cs, toggles


def trajectory(speed=17.0, accel=0.4, allow_throttle=False):
  return SimpleNamespace(speeds=[speed + accel * t for t in CONTROL_N_T_IDX], accels=[accel] * len(CONTROL_N_T_IDX), allowThrottle=allow_throttle)


def test_i30_old_long_respects_throttle_veto_without_integral_windup():
  controller, cs, toggles = interceptor_controller()
  for _ in range(500):
    assert controller.update_old_long(True, cs, trajectory(), [-3.5, 2.0], 0.0, toggles) == 0.0
  assert controller.pid.i == pytest.approx(0.25)
  assert controller.update_old_long(True, cs, trajectory(allow_throttle=True), [-3.5, 2.0], 0.0, toggles) > 0.0


def test_throttle_veto_preserves_negative_lead_request():
  controller, cs, toggles = interceptor_controller()
  assert controller.update_old_long(True, cs, trajectory(14.0, -1.0), [-3.5, 2.0], 0.0, toggles) < -0.5


@pytest.mark.parametrize("car_name,interceptor", [("toyota", True), ("i30", False)])
def test_throttle_veto_change_is_scoped_to_i30_interceptor(car_name, interceptor):
  controller, cs, toggles = interceptor_controller(car_name, interceptor)
  assert controller.update_old_long(True, cs, trajectory(), [-3.5, 2.0], 0.0, toggles) > 0.0


def test_throttle_veto_also_covers_starting_state():
  controller, cs, toggles = interceptor_controller()
  controller.CP.startingState = True
  controller.long_control_state = LongCtrlState.stopping
  assert controller.update_old_long(True, cs, trajectory(), [-3.5, 2.0], 0.0, toggles) == 0.0
  assert controller.long_control_state == LongCtrlState.starting


def test_inactive_i30_still_resets_output_and_integral():
  controller, cs, toggles = interceptor_controller()
  assert controller.update_old_long(False, cs, trajectory(), [-3.5, 2.0], 0.0, toggles) == 0.0
  assert controller.pid.i == 0.0


# --- i30 pedal map integration (stub map, independent of the fitted calibration) ---

class StubMap:
  """Linear plant: accel = 2.0 * command - 0.4 (coast -0.4), valid unless `available` is False."""

  def __init__(self):
    self.available = True
    self.last_limited = False

  def command(self, accel, v_ego, rpm, pitch):
    if not self.available:
      return None
    raw = (accel + 0.4) / 2.0
    self.last_limited = raw >= 0.8
    return min(max(raw, 0.0), 0.8)

  def accel(self, command, v_ego, rpm, pitch):
    return 2.0 * command - 0.4 if self.available else None


def map_controller():
  controller, cs, toggles = interceptor_controller()
  controller.pedal_map, controller.legacy_pedal_accel = StubMap(), 1.6
  controller.pid.i = 0.0
  cs.engineRpm = 1600.0
  toggles.max_desired_acceleration = 4.0
  return controller, cs, toggles


def run(controller, cs, toggles, plan, n=1, active=True):
  for _ in range(n):
    controller.update_old_long(active, cs, plan, [-3.5, 2.0], 0.0, toggles, pitch=0.0)
  return controller.pedal_command


def test_map_gives_pedal_for_mild_decel_but_veto_is_exact_zero():
  controller, cs, toggles = map_controller()
  cruise = trajectory(15.0, 0.0, allow_throttle=True)
  pedal = run(controller, cs, toggles, cruise, 50)
  assert controller.pedal_from_map and pedal == pytest.approx(0.2, abs=0.02)  # road load, not zero
  assert run(controller, cs, toggles, trajectory(15.0, 0.0, allow_throttle=False)) == 0.0


def test_map_inactive_is_exact_zero_and_reengage_starts_from_reset():
  controller, cs, toggles = map_controller()
  run(controller, cs, toggles, trajectory(17.0, 0.4, allow_throttle=True), 100)
  assert run(controller, cs, toggles, trajectory(17.0, 0.4, allow_throttle=True), active=False) == 0.0
  assert controller.pid.i == 0.0 and not controller.pedal_from_map
  run(controller, cs, toggles, trajectory(15.0, 0.0, allow_throttle=True))
  assert abs(controller.pid.i) < 1e-3  # no transfer from the inactive zero pedal


def test_map_fallback_is_legacy_and_bumpless():
  controller, cs, toggles = map_controller()
  plan = trajectory(15.5, 0.0, allow_throttle=True)
  before = run(controller, cs, toggles, plan, 200)
  controller.pedal_map.available = False
  after = run(controller, cs, toggles, plan)
  assert not controller.pedal_from_map
  assert abs(after - before) < 0.01  # slew limited and integral transferred
  settled = run(controller, cs, toggles, plan, 150)
  assert abs(settled - before) < 0.05  # legacy now holds the same pedal through its integral
  assert settled == pytest.approx(controller.last_output_accel / 1.6, abs=1e-6)
  controller.pedal_map.available = True
  pedals = [run(controller, cs, toggles, plan)]
  assert not controller.pedal_from_map  # re-entry needs the map continuously available for a while
  pedals += [run(controller, cs, toggles, plan) for _ in range(80)]
  assert controller.pedal_from_map
  assert max(abs(b - a) for a, b in zip(pedals[:-1], pedals[1:], strict=True)) < 0.01


def test_coverage_chatter_at_saturation_does_not_wind_integral_negative():
  controller, cs, toggles = map_controller()
  slow = SimpleNamespace(**{**vars(cs), 'vEgo': 8.0})
  plan = trajectory(15.0, 1.5, allow_throttle=True)  # large request: map saturates at 0.8
  for k in range(1500):
    controller.pedal_map.available = (k // 30) % 2 == 0  # map coverage flickers every 0.3 s
    run(controller, slow, toggles, plan)
    assert controller.pid.i >= -1e-9
  assert controller.pedal_command >= 0.79


def test_map_integral_bounded_and_frozen_at_zero_pedal():
  controller, cs, toggles = map_controller()
  fast = SimpleNamespace(**{**vars(cs), 'vEgo': 20.0})
  plan = trajectory(15.0, 0.0, allow_throttle=True)
  run(controller, fast, toggles, plan, 3)
  assert controller.pedal_command == 0.0 and controller.pedal_saturation == -1
  i0 = controller.pid.i
  run(controller, fast, toggles, plan, 500)
  assert controller.pid.i == pytest.approx(i0)  # cannot act on the error: no windup
  slow = SimpleNamespace(**{**vars(cs), 'vEgo': 10.0})
  run(controller, slow, toggles, trajectory(15.0, 0.0, allow_throttle=True), 3000)
  assert controller.pid.i <= 0.5 + 1e-9


def test_map_integral_frozen_at_pedal_limit():
  controller, cs, toggles = map_controller()
  slow = SimpleNamespace(**{**vars(cs), 'vEgo': 8.0})
  run(controller, slow, toggles, trajectory(15.0, 1.5, allow_throttle=True), 3)
  assert controller.pedal_saturation == 1
  i0 = controller.pid.i
  run(controller, slow, toggles, trajectory(15.0, 1.5, allow_throttle=True), 300)
  assert controller.pid.i == pytest.approx(i0)


def test_map_respects_max_desired_acceleration():
  controller, cs, toggles = map_controller()
  toggles.max_desired_acceleration = 0.1
  pedal = run(controller, cs, toggles, trajectory(18.0, 1.0, allow_throttle=True), 20)
  assert pedal <= (0.1 + 0.4) / 2.0 + 1e-9
