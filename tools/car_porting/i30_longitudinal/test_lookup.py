import math

import pytest

from openpilot.tools.car_porting.i30_longitudinal.lookup import PedalMap, coast_state_candidate


@pytest.fixture
def lookup():
  return PedalMap(
    dict(
      pedal=[0.0, 0.5, 1.0],
      pitch_gain=1.0,
      max_abs_pitch=0.1,
      gears={
        '5': dict(speed=[10.0, 20.0], accel=[[-0.3, 0.7, 1.7], [-0.5, 0.5, 1.5]], supported=[[True, True], [True, True]]),
      },
    )
  )


def test_physical_zero_acceleration_needs_positive_pedal(lookup):
  assert lookup.acceleration(5, 15.0, 0.0, 0.0) == pytest.approx(-0.4)
  assert lookup.pedal(5, 15.0, 0.0, 0.0) == pytest.approx(0.2)
  assert lookup.pedal(5, 15.0, 0.0, -2.0) == 0.0


def test_inversion_and_monotonicity_across_speed_pitch(lookup):
  for speed in [10.0, 12.0, 15.0, 19.0, 20.0]:
    for pitch in [-0.09, 0.0, 0.09]:
      accelerations = [lookup.acceleration(5, speed, pitch, p / 20) for p in range(21)]
      assert all(a < b for a, b in zip(accelerations, accelerations[1:], strict=False))
      for pedal, acceleration in enumerate(accelerations):
        assert lookup.pedal(5, speed, pitch, acceleration) == pytest.approx(pedal / 20)


def test_grade_sign_and_unsupported_inputs(lookup):
  assert lookup.acceleration(5, 15.0, 0.05, 0.2) < lookup.acceleration(5, 15.0, -0.05, 0.2)
  for gear, speed, pitch in [(0, 15.0, 0.0), (3, 15.0, 0.0), (5, 9.0, 0.0), (5, 21.0, 0.0), (5, 15.0, 0.2), (5, math.nan, 0.0)]:
    assert lookup.pedal(gear, speed, pitch, 0.0) is None
  assert lookup.pedal(5, 15.0, 0.0, 5.0) is None
  assert lookup.pedal(5, 15.0, 0.0, math.nan) is None
  assert lookup.acceleration(5, 15.0, 0.0, -1.0) is None
  lookup.calibration['gears']['5']['supported'][1][1] = False
  assert lookup.pedal(5, 15.0, 0.0, 1.0) is None
  assert lookup.acceleration(5, 15.0, 0.0, 0.8) is None


def test_flat_response_cannot_amplify_small_acceleration_errors(lookup):
  lookup.calibration['gears']['5']['accel'] = [[-0.3, -0.29, 1.0], [-0.3, -0.29, 1.0]]
  assert lookup.pedal(5, 15.0, 0.0, -0.295) is None


def test_state_gap_candidate_only_limits_internal_state():
  assert coast_state_candidate(17.0, -2.0, 20.0, -0.35, 1.0, True, 25.0) == pytest.approx((19.5, -0.5))
  assert coast_state_candidate(19.8, -0.2, 20.0, -0.35, 1.0, True, 25.0) == (19.8, -0.2)
  # Downhill cannot initialize a positive acceleration merely from coast capability.
  assert coast_state_candidate(17.0, -2.0, 20.0, 0.2, 1.0, True, 25.0) == pytest.approx((19.5, -0.15))


@pytest.mark.parametrize(
  'kwargs',
  [
    {'valid': False},
    {'force_decel': True},
    {'zero_pedal_seconds': 0.49},
    {'measured_speed': 7.9},
    {'cruise_speed': 15.0},
    {'coast_accel': math.nan},
  ],
)
def test_state_candidate_keeps_cruise_drop_and_invalid_cases(kwargs):
  args = dict(speed=17.0, accel=-2.0, measured_speed=20.0, coast_accel=-0.35, zero_pedal_seconds=1.0, valid=True, cruise_speed=25.0)
  args.update(kwargs)
  assert coast_state_candidate(**args) == (17.0, -2.0)
