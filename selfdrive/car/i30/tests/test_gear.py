from openpilot.selfdrive.car.i30.gear import GearEstimator


def observe(estimator, ratio, start=1, count=4):
  gear = 0
  for sample in range(start, start + count):
    gear = estimator.update(20.0, ratio * 20.0, sample * 50_000_000, valid=True)
  return gear


def test_calibrated_gears_require_fresh_confirmations():
  for gear, ratio in ((4, 130.6), (5, 105.7), (6, 92.1)):
    estimator = GearEstimator()
    assert observe(estimator, ratio, count=3) == 0
    assert observe(estimator, ratio, start=4, count=1) == gear


def test_repeated_can_sample_does_not_confirm_gear():
  estimator = GearEstimator()
  for _ in range(20):
    assert estimator.update(20.0, 2114.0, 50_000_000, valid=True) == 0
  assert observe(estimator, 105.7, start=2, count=3) == 5


def test_shift_invalidates_gear_and_requires_reconfirmation():
  estimator = GearEstimator()
  assert observe(estimator, 130.6) == 4
  assert observe(estimator, 118.0, start=5, count=1) == 0
  assert observe(estimator, 105.7, start=6, count=3) == 0
  assert observe(estimator, 105.7, start=9, count=1) == 5
  # Even a direct jump between calibrated ratios starts a new confirmation.
  assert observe(estimator, 92.1, start=10, count=3) == 0
  assert observe(estimator, 92.1, start=13, count=1) == 6


def test_unmatched_and_uncalibrated_ratios_stay_unknown():
  for ratio in (0.0, 70.0, 81.0, 99.0, 118.0, 150.0, 200.0, 350.0):
    assert observe(GearEstimator(), ratio) == 0


def test_unreliable_inputs_clear_previous_estimate():
  for speed, rpm, timestamp, valid in (
    (1.9, 800.0, 250_000_000, True),
    (20.0, 699.0, 250_000_000, True),
    (20.0, 2114.0, 250_000_000, False),  # Clutch, reverse, or invalid CAN.
    (20.0, 2114.0, 0, True),
    (float("nan"), 2114.0, 250_000_000, True),
    (20.0, float("inf"), 250_000_000, True),
  ):
    estimator = GearEstimator()
    assert observe(estimator, 105.7) == 5
    assert estimator.update(speed, rpm, timestamp, valid) == 0
    assert observe(estimator, 105.7, start=6, count=3) == 0
    assert observe(estimator, 105.7, start=9, count=1) == 5


def test_backward_timestamp_requires_reconfirmation():
  estimator = GearEstimator()
  assert observe(estimator, 105.7) == 5
  assert observe(estimator, 105.7, start=1, count=3) == 0
  assert observe(estimator, 105.7, start=4, count=1) == 5
