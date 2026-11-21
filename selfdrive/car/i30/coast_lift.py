"""Early lift-off for the pedal-only i30: a throttle veto from known coast capability.

The car cannot brake, so throttle must be released before the deceleration needed to
settle behind a lead exceeds what coasting in the current gear can deliver. This only
removes throttle (via longitudinalPlan.allowThrottle); it never edits the MPC trajectory,
lead model or stop request, and it cannot add braking authority.
"""
from math import isfinite

from openpilot.common.realtime import DT_MDL

# Assume weak coasting when the map has no estimate (gear unknown, outside coverage).
DEFAULT_COAST_ACCEL = -0.15  # m/s^2
MIN_CAPABILITY = 0.05  # m/s^2; a downhill coast that does not decelerate still vetoes throttle
LIFT_FRACTION = 0.7  # lift when needed decel reaches this fraction of coast capability (sim sweep 0.3-1.0)
RELEASE_FRACTION = 0.28  # release once needed decel falls below this fraction
MIN_SPEED = 5.0  # m/s; below this the clutch logic disengages anyway
STOP_DISTANCE = 6.0  # m, same as long_mpc
# Pedal command to wheel response: ~0.1 s dead time plus ~0.35 s torque rise (fitted).
LIFT_DELAY = 0.5  # s

# Steady following sits a few metres inside t_follow * v_lead + STOP_DISTANCE. There the
# old -10 cliff for a used-up margin flipped on every positive closing sample of the noisy
# vision lead (~690 vetoes/h of 0.1 s on the 2026-11-20 drives). Near the desired gap the
# stopping distance is floored instead, so the need stays continuous.
MARGIN_FLOOR = 3.0  # m
# Well inside the desired gap, any sustained closing vetoes throttle at once: the floor must
# not make a 10 m gap look as relaxed as a 30 m one.
SHORT_HEADWAY_FRACTION = 0.7  # of the desired distance
SHORT_HEADWAY_CLOSING = 0.3  # m/s
# While the gap is opening, a braking lead counts in full at zero closing speed and fades
# out by this opening speed, so equal speed behind a braking lead is not a blind spot.
OPENING_REF = 1.0  # m/s
# Vision lead speed jumps by >0.5 m/s between frames; a short low-pass on the need removes
# single-frame vetoes. Severe needs bypass it, and a lead that jumps closer reseeds it.
NEEDED_FILTER_TAU = 0.3  # s
SEVERE_NEED_FACTOR = 3.0
LEAD_JUMP = 5.0  # m of distance decrease between updates
SEVERE_NEED = -10.0  # m/s^2


def needed_accel(distance, v_ego, v_lead, a_lead, desired_distance, delay):
  """Constant ego acceleration that brings closing speed to zero at the desired gap.

  Closing continues for `delay` seconds before any change takes effect. Lead
  deceleration (not acceleration) is assumed to continue. The stopping distance is
  floored at MARGIN_FLOOR, and SHORT_HEADWAY_FRACTION of the desired distance with
  sustained closing returns SEVERE_NEED.
  """
  closing = v_ego - v_lead
  margin = distance - desired_distance - max(closing, 0.0) * delay
  lead_braking = min(a_lead, 0.0)
  if closing <= 0.0:
    if margin < 0.0:
      return lead_braking
    return lead_braking * max(0.0, 1.0 + closing / OPENING_REF)
  if distance < SHORT_HEADWAY_FRACTION * desired_distance and closing > SHORT_HEADWAY_CLOSING:
    return SEVERE_NEED
  return lead_braking - closing * closing / (2.0 * max(margin, MARGIN_FLOOR))


class CoastLiftOff:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.active = False
    self.needed = 0.0
    self.raw_needed = 0.0
    self.capability = 0.0
    self._filter_valid = False
    self._last_distance = 0.0

  def update(self, lead_valid, v_ego, distance, v_lead, a_lead, t_follow, coast_accel, delay=LIFT_DELAY):
    if not lead_valid or not all(isfinite(x) for x in (v_ego, distance, v_lead, a_lead, t_follow, delay)) or v_ego < MIN_SPEED:
      self.active = False
      self._filter_valid = False
      return False
    coast = coast_accel if coast_accel is not None and isfinite(coast_accel) else DEFAULT_COAST_ACCEL
    self.capability = max(-coast, MIN_CAPABILITY)
    desired = t_follow * v_lead + STOP_DISTANCE
    self.raw_needed = needed_accel(distance, v_ego, v_lead, a_lead, desired, delay)

    severe = self.raw_needed < -SEVERE_NEED_FACTOR * LIFT_FRACTION * self.capability
    jumped_closer = self._filter_valid and distance < self._last_distance - LEAD_JUMP
    if not self._filter_valid or severe or jumped_closer:
      self.needed = self.raw_needed
    else:
      self.needed += self.dt / (NEEDED_FILTER_TAU + self.dt) * (self.raw_needed - self.needed)
    self._filter_valid = True
    self._last_distance = distance

    if self.active:
      self.active = self.needed < -RELEASE_FRACTION * self.capability
    else:
      self.active = self.needed < -LIFT_FRACTION * self.capability
    return self.active
