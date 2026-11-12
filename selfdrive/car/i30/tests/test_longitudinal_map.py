import json
import math

import pytest

from openpilot.selfdrive.car.i30.coast_lift import CoastLiftOff, needed_accel
from openpilot.selfdrive.car.i30.longitudinal_map import CALIBRATION_PATH, I30LongitudinalMap

K = 1.5e-4


def synthetic(saturates=True):
  pedal = [0.0, 10.0, 20.0, 30.0, 50.0]
  rpm = [1000.0, 2000.0, 3000.0]
  row = [-10.0, 15.0, 35.0, 50.0, 51.0]
  return dict(
    pedal_bp=pedal, rpm_bp=rpm, torque=[row, row, row], max_pedal=[30.0, 30.0, 30.0], saturates=[saturates] * 3,
    supported=[[True, True, True, True]] * 3,
    gears={'5': dict(ratio=105.7, tol=6.4, k=K, controllable=True), '3': dict(ratio=173.0, tol=5.0, k=K, controllable=False)},
    drag=dict(c0=0.05, c2=0.0004), grade=dict(gain=1.0, offset=0.0, max_abs_pitch=0.09),
    pedal_command=dict(scale=1.0, offset=0.0), limits=dict(v_min=5.0, v_max=40.0),
  )


def at(m, accel=0.0, v=20.0, gear_ratio=105.7, pitch=0.0):
  return m.command(accel, v, gear_ratio * v, pitch)


def test_forward_inverse_consistent():
  m = I30LongitudinalMap(synthetic())
  for a in (-0.3, 0.0, 0.2, 0.5):
    cmd = at(m, a)
    assert cmd is not None
    assert m.accel(cmd, 20.0, 105.7 * 20.0, 0.0) == pytest.approx(a, abs=1e-6)


def test_monotone_in_request():
  m = I30LongitudinalMap(synthetic())
  cmds = [at(m, a / 20) for a in range(-12, 20)]
  assert all(b >= a for a, b in zip(cmds, cmds[1:], strict=False))


def test_below_coast_is_exact_zero():
  m = I30LongitudinalMap(synthetic())
  coast = m.coast_accel(20.0, 2114.0, 0.0)
  assert coast < 0
  assert at(m, coast - 0.01) == 0.0
  assert at(m, -3.5) == 0.0
  assert at(m, coast + 0.05) > 0.0


def test_grade_feedforward():
  m = I30LongitudinalMap(synthetic())
  assert at(m, 0.0, pitch=0.03) > at(m, 0.0) > at(m, 0.0, pitch=-0.02)
  assert m.coast_accel(20.0, 2114.0, -0.05) > m.coast_accel(20.0, 2114.0, 0.0)


def test_unknown_gear_and_speed_fall_back():
  m = I30LongitudinalMap(synthetic())
  assert at(m, 0.2, gear_ratio=140.0) is None  # between gear windows (clutch / shift)
  assert at(m, 0.2, gear_ratio=173.0) is None  # calibrated offline, not controllable
  assert at(m, 0.2, v=3.0) is None
  assert m.command(0.2, 20.0, 5000.0, 0.0) is None  # rpm outside table
  assert m.command(float('nan'), 20.0, 2114.0, 0.0) is None
  assert m.command(0.2, float('nan'), 2114.0, 0.0) is None


def test_saturation_holds_knee_but_coverage_edge_falls_back():
  sat = I30LongitudinalMap(synthetic(saturates=True))
  assert at(sat, 3.0) == pytest.approx(0.30) and sat.last_limited
  at(sat, 0.1)
  assert not sat.last_limited
  edge = I30LongitudinalMap(synthetic(saturates=False))
  assert at(edge, 3.0) is None
  assert at(edge, 0.1) is not None


def test_unsupported_cell_falls_back():
  cal = synthetic()
  cal['supported'] = [[True, False, True, True]] * 3
  m = I30LongitudinalMap(cal)
  assert at(m, -0.4) is not None  # lowest cell
  needed_in_cell_1 = I30LongitudinalMap(synthetic()).accel(0.15, 20.0, 2114.0, 0.0)
  assert m.command(needed_in_cell_1, 20.0, 2114.0, 0.0) is None
  assert m.accel(0.15, 20.0, 2114.0, 0.0) is None


def test_command_offset_dead_band():
  cal = synthetic()
  cal['pedal_command'] = dict(scale=1.0, offset=0.9)
  m = I30LongitudinalMap(cal)
  assert m.ecu_to_command(0.5) == 0.0
  assert m.command_to_ecu(0.0) == 0.0
  assert m.ecu_to_command(m.command_to_ecu(0.2)) == pytest.approx(0.2)


def test_rejects_non_monotone_table():
  cal = synthetic()
  cal['torque'][1] = [-10.0, 15.0, 14.0, 50.0, 51.0]
  with pytest.raises(ValueError):
    I30LongitudinalMap(cal)


def test_shipped_calibration_loads_and_is_sane():
  cal = json.loads(CALIBRATION_PATH.read_text())
  m = I30LongitudinalMap(cal)
  assert set(m.gears) <= {4, 5, 6}
  for ratio, _, k in m.gears.values():
    assert k > 0 and 85 < ratio < 140
  # Zero request at cruise needs some pedal; strong decel request gives none.
  for v, g in ((14.0, 4), (20.0, 5), (25.0, 6)):
    if g not in m.gears:
      continue
    rpm = m.gears[g][0] * v
    coast = m.coast_accel(v, rpm, 0.0)
    if coast is None:
      continue
    assert -1.0 < coast < 0.0
    assert m.command(-2.0, v, rpm, 0.0) == 0.0


# --- early lift-off ---------------------------------------------------------

def test_needed_accel_kinematics():
  # 10 m/s closing with 50 m of margin and no delay: 1 m/s^2.
  assert needed_accel(80.0, 25.0, 15.0, 0.0, 30.0, 0.0) == pytest.approx(-1.0)
  assert needed_accel(80.0, 15.0, 25.0, 0.0, 30.0, 0.5) == 0.0
  assert needed_accel(31.0, 25.0, 15.0, 0.0, 30.0, 0.5) == -10.0


def test_lift_off_hysteresis_and_capability():
  lift = CoastLiftOff()
  # Far, slowly closing: no veto.
  assert not lift.update(True, 22.0, 120.0, 21.0, 0.0, 1.45, -0.4)
  # Close and closing: needed decel exceeds half of coast capability -> veto.
  assert lift.update(True, 22.0, 60.0, 18.0, 0.0, 1.45, -0.4)
  # Need between release (0.08) and lift (0.2) thresholds: stays latched, but would not lift fresh.
  assert lift.update(True, 22.0, 60.0, 19.5, 0.0, 1.45, -0.4)
  assert not CoastLiftOff().update(True, 22.0, 60.0, 19.5, 0.0, 1.45, -0.4)
  assert not lift.update(True, 22.0, 70.0, 22.0, 0.0, 1.45, -0.4)


def test_weaker_coast_lifts_earlier():
  strong, weak = CoastLiftOff(), CoastLiftOff()
  args = (True, 22.0, 75.0, 19.0, 0.0, 1.45)
  assert weak.update(*args, -0.15) and not strong.update(*args, -0.8)


def test_unknown_coast_uses_weak_default_and_invalid_lead_clears():
  lift = CoastLiftOff()
  assert lift.update(True, 22.0, 75.0, 19.0, 0.0, 1.45, None)
  assert not lift.update(False, 22.0, 75.0, 19.0, 0.0, 1.45, None)
  assert not lift.update(True, 3.0, 10.0, 0.0, 0.0, 1.45, -0.4)
  assert not lift.update(True, 22.0, math.nan, 19.0, 0.0, 1.45, -0.4)
