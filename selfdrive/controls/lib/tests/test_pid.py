import pytest

from openpilot.selfdrive.controls.lib.pid import PIDController


class TestPIDController:
  def test_integrator_floor(self):
    pid = PIDController(0.0, 1.0, rate=1)

    output = pid.update(-1.0, integrator_min=0.0)

    assert output == pytest.approx(0.0)
    assert pid.i == pytest.approx(0.0)

  def test_integrator_floor_allows_positive_integral_to_unwind_to_zero(self):
    pid = PIDController(0.0, 1.0, rate=1)
    pid.update(1.0, integrator_min=0.0)

    for _ in range(2):
      pid.update(-1.0, integrator_min=0.0)

    assert pid.i == pytest.approx(0.0)

  def test_integrator_floor_applies_while_frozen(self):
    pid = PIDController(0.0, 1.0, rate=1)
    pid.i = -1.0

    pid.update(0.0, freeze_integrator=True, integrator_min=0.0)

    assert pid.i == pytest.approx(0.0)
