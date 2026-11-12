"""Inactive, scalar early-lift experiment for a pedal-only car."""

from math import isfinite


class EarlyCoast:
  def __init__(self):
    self.blocked = False

  def update(self, valid, speed, distance, relative_speed, lead_accel, coast_accel, follow_time, actuator_delay):
    if not valid or not all(isfinite(x) for x in (speed, distance, relative_speed, lead_accel, coast_accel, follow_time, actuator_delay)) or speed < 8:
      self.blocked = False
      return False
    # Predict one second of continued lead deceleration, not a full lead stop.
    closing = max(0.0, -relative_speed - min(lead_accel, 0.0))
    # A non-decelerating downhill coast needs lift-off, but cannot promise a stop.
    capability = max(0.05, -coast_accel)
    closing_loss = closing * max(actuator_delay, 0.0) + closing * closing / (2 * capability)
    threshold = 6.0 + max(follow_time, 0.0) * speed + closing_loss
    if self.blocked:
      self.blocked = closing > 0.05 and distance < threshold + 5.0
    else:
      self.blocked = closing > 0.2 and distance < threshold
    return self.blocked
