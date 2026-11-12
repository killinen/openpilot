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
