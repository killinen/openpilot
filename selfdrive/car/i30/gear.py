from math import isfinite


# Gear, engine RPM / vehicle speed (m/s), clutch-detection tolerance.
I30_GEAR_RATIO_WINDOWS = (
  (4, 130.6, 10.0),
  (5, 105.7, 8.0),
  (6, 92.1, 6.0),
)
I30_MIN_CLUTCH_SPEED = 2.0
I30_MIN_CLUTCH_RPM = 700.0
I30_CLUTCH_RELEASE_TOL_MULTIPLIER = 0.8
I30_GEAR_CONFIRM_SAMPLES = 4


class GearEstimator:
  """Estimate only the calibrated forward gears; zero means unknown."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.candidate = 0
    self.samples = 0
    self.last_sample_time_nanos = 0

  def update(self, v_ego, engine_rpm, sample_time_nanos, valid):
    if (not valid or sample_time_nanos <= 0 or not isfinite(v_ego) or not isfinite(engine_rpm) or
        v_ego < I30_MIN_CLUTCH_SPEED or engine_rpm < I30_MIN_CLUTCH_RPM):
      self.reset()
      return 0

    ratio = engine_rpm / v_ego
    gear, closest_ratio, tolerance = min(I30_GEAR_RATIO_WINDOWS, key=lambda entry: abs(ratio - entry[1]))
    # Use the tighter clutch-release window, not the broad clutch-press window.
    if abs(ratio - closest_ratio) >= tolerance * I30_CLUTCH_RELEASE_TOL_MULTIPLIER:
      self.reset()
      return 0

    if gear != self.candidate or sample_time_nanos < self.last_sample_time_nanos:
      self.reset()
      self.candidate = gear

    # A held CAN value must not count as multiple observations at control rate.
    if sample_time_nanos > self.last_sample_time_nanos:
      self.samples = min(self.samples + 1, I30_GEAR_CONFIRM_SAMPLES)
      self.last_sample_time_nanos = sample_time_nanos

    return gear if self.samples >= I30_GEAR_CONFIRM_SAMPLES else 0
