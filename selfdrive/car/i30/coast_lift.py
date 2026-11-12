"""Early lift-off for the pedal-only i30: a throttle veto from known coast capability.

The car cannot brake, so throttle must be released before the deceleration needed to
settle behind a lead exceeds what coasting in the current gear can deliver. This only
removes throttle (via longitudinalPlan.allowThrottle); it never edits the MPC trajectory,
lead model or stop request, and it cannot add braking authority.
"""
from math import isfinite

# Assume weak coasting when the map has no estimate (gear unknown, outside coverage).
DEFAULT_COAST_ACCEL = -0.15  # m/s^2
MIN_CAPABILITY = 0.05  # m/s^2; a downhill coast that does not decelerate still vetoes throttle
LIFT_FRACTION = 0.7  # lift when needed decel reaches this fraction of coast capability (sim sweep 0.3-1.0)
RELEASE_FRACTION = 0.28  # release once needed decel falls below this fraction
MIN_SPEED = 5.0  # m/s; below this the clutch logic disengages anyway
STOP_DISTANCE = 6.0  # m, same as long_mpc
# Pedal command to wheel response: ~0.1 s dead time plus ~0.35 s torque rise (fitted).
LIFT_DELAY = 0.5  # s


def needed_accel(distance, v_ego, v_lead, a_lead, desired_distance, delay):
  """Constant ego acceleration that brings closing speed to zero at the desired gap.

  Closing continues for `delay` seconds before any change takes effect. Lead
  deceleration (not acceleration) is assumed to continue. Returns a large negative
  value once the margin is already used up.
  """
  closing = v_ego - v_lead
  margin = distance - desired_distance - max(closing, 0.0) * delay
  if closing <= 0.0:
    return min(a_lead, 0.0) if margin < 0.0 else 0.0
  if margin <= 0.5:
    return -10.0
  return min(a_lead, 0.0) - closing * closing / (2.0 * margin)


class CoastLiftOff:
  def __init__(self):
    self.active = False
    self.needed = 0.0
    self.capability = 0.0

  def update(self, lead_valid, v_ego, distance, v_lead, a_lead, t_follow, coast_accel, delay=LIFT_DELAY):
    if not lead_valid or not all(isfinite(x) for x in (v_ego, distance, v_lead, a_lead, t_follow, delay)) or v_ego < MIN_SPEED:
      self.active = False
      return False
    coast = coast_accel if coast_accel is not None and isfinite(coast_accel) else DEFAULT_COAST_ACCEL
    self.capability = max(-coast, MIN_CAPABILITY)
    desired = t_follow * v_lead + STOP_DISTANCE
    self.needed = needed_accel(distance, v_ego, v_lead, a_lead, desired, delay)
    if self.active:
      self.active = self.needed < -RELEASE_FRACTION * self.capability
    else:
      self.active = self.needed < -LIFT_FRACTION * self.capability
    return self.active
