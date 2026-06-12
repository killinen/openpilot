import pytest

from openpilot.selfdrive.controls.lib.longcontrol import gas_interceptor_integrator_limits


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
