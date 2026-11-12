"""Small offline/shadow candidates. These are NOT connected to vehicle control.

Fitting is workstation-only. Evaluation uses scalar math, two table rows and at
most eight pedal knots; it has no NumPy, SciPy, logging or learning dependency.
"""

from bisect import bisect_right
from math import isfinite, sin


def bracket(points, value):
  index = max(0, min(len(points) - 2, bisect_right(points, value) - 1))
  fraction = (value - points[index]) / (points[index + 1] - points[index])
  return index, fraction


class PedalMap:
  def __init__(self, calibration):
    self.calibration = calibration

  def curve(self, gear, speed, pitch):
    model = self.calibration['gears'].get(str(gear))
    if model is None or not all(isfinite(x) for x in (speed, pitch)):
      return None
    speeds = model['speed']
    if not speeds[0] <= speed <= speeds[-1] or abs(pitch) > self.calibration['max_abs_pitch']:
      return None
    index, weight = bracket(speeds, speed)
    grade = -9.81 * sin(pitch) * self.calibration['pitch_gain']
    curve = [a + weight * (b - a) + grade for a, b in zip(model['accel'][index], model['accel'][index + 1], strict=True)]
    support = [a and b for a, b in zip(model['supported'][index], model['supported'][index + 1], strict=True)]
    return curve, support

  def acceleration(self, gear, speed, pitch, pedal):
    result = self.curve(gear, speed, pitch)
    if result is None or not isfinite(pedal) or not 0 <= pedal <= 1:
      return None
    curve, support = result
    index, weight = bracket(self.calibration['pedal'], pedal)
    if not support[index]:
      return None
    return curve[index] + weight * (curve[index + 1] - curve[index])

  def pedal(self, gear, speed, pitch, acceleration):
    """Return None outside observed coverage, never extrapolate missing cells."""
    result = self.curve(gear, speed, pitch)
    if result is None or not isfinite(acceleration):
      return None
    curve, support = result
    if acceleration <= curve[0]:
      return 0.0 if support[0] else None
    if acceleration > curve[-1]:
      return None
    index, weight = bracket(curve, acceleration)
    if not support[index]:
      return None
    pedal = self.calibration['pedal']
    # A nearly flat response is not a usable inverse: tiny acceleration errors
    # would cause a large pedal jump. Fall back rather than amplify that error.
    slope = (curve[index + 1] - curve[index]) / (pedal[index + 1] - pedal[index])
    if slope < self.calibration.get('min_inverse_slope', 0.2):
      return None
    return pedal[index] + weight * (pedal[index + 1] - pedal[index])


def coast_state_candidate(speed, accel, measured_speed, coast_accel, zero_pedal_seconds, valid, cruise_speed, force_decel=False, speed_gap=0.5, accel_gap=0.15):
  """Shadow experiment on MPC initial state, NOT a floor on lead decel demand.

  Do not call this on the published trajectory or controller output. The original
  lead request/stop request must survive. Cruise-cap and forced-decel cases retain
  their existing state. Integration requires an MPC/closed-loop evaluation first.
  """
  if (
    not valid
    or force_decel
    or not all(isfinite(x) for x in (speed, accel, measured_speed, coast_accel, zero_pedal_seconds, cruise_speed))
    or zero_pedal_seconds < 0.5
    or measured_speed < 8.0
    or cruise_speed < measured_speed - speed_gap
  ):
    return speed, accel
  return max(speed, measured_speed - speed_gap), max(accel, min(coast_accel, 0.0) - accel_gap)
