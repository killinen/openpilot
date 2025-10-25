from cereal import log
from common.conversions import Conversions as CV
from common.params import Params
from common.realtime import DT_MDL, sec_since_boot

FOOT_TO_METER = 0.3048
LANE_CHANGE_DELAY_DEFAULT = 2.0
LANE_DETECTION_WIDTH_DEFAULT_METERS = 2.7  # ~9 ft
PARAM_REFRESH_TIME = 1.0

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 30 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
  },
}

class LaneChangeConfig:
  def __init__(self):
    self.params = Params()
    self.last_update = -1.0

    self.nudgeless = False
    self.lane_detection_width = LANE_DETECTION_WIDTH_DEFAULT_METERS
    self.lane_detection = True
    self.lane_change_delay = LANE_CHANGE_DELAY_DEFAULT
    self.minimum_lane_change_speed = LANE_CHANGE_SPEED_MIN
    self.one_lane_change = False
    self.is_metric = self.params.get_bool("IsMetric")

  def _get_float(self, key, default):
    value = self.params.get(key)
    if value is None:
      return default

    try:
      return float(value.decode('utf-8'))
    except (ValueError, AttributeError):
      return default

  def update(self):
    now = sec_since_boot()
    if self.last_update >= 0 and now - self.last_update < PARAM_REFRESH_TIME:
      return

    self.last_update = now
    self.is_metric = self.params.get_bool("IsMetric")

    # Toggle flags
    self.nudgeless = self.params.get_bool("NudgelessLaneChange")

    # Lane change delay (seconds)
    self.lane_change_delay = max(0.0, self._get_float("LaneChangeTime", LANE_CHANGE_DELAY_DEFAULT))

    # Minimum lane change speed is stored in km/h
    default_min_speed_kph = 32.0 if self.nudgeless else (LANE_CHANGE_SPEED_MIN * CV.MS_TO_KPH)
    min_speed_kph = self._get_float("MinimumLaneChangeSpeed", default_min_speed_kph)
    self.minimum_lane_change_speed = max(0.0, min_speed_kph) * CV.KPH_TO_MS

    # Lane detection width (stored in user units; convert to meters)
    default_width_raw = LANE_DETECTION_WIDTH_DEFAULT_METERS / (1.0 if self.is_metric else FOOT_TO_METER)
    lane_detection_raw = self._get_float("LaneDetectionWidth", default_width_raw)
    width_m = lane_detection_raw * (1.0 if self.is_metric else FOOT_TO_METER)
    self.lane_detection_width = max(0.0, width_m)
    self.lane_detection = self.nudgeless and self.lane_detection_width > 0.0

    # Allow one lane change per signal by default when nudgeless is active
    one_lane_raw = self.params.get("OneLaneChange")
    one_lane_enabled = True if one_lane_raw is None else one_lane_raw != b"0"
    self.one_lane_change = self.nudgeless and one_lane_enabled


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none

    self.lane_change_completed = False
    self.lane_change_wait_timer = 0.0
    self.config = LaneChangeConfig()

  def update(self, carstate, active, lane_change_prob, lane_width_left=0.0, lane_width_right=0.0):
    self.config.update()

    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < self.config.minimum_lane_change_speed

    if not active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.lane_change_wait_timer += DT_MDL

        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        # Driver torque request takes priority
        manual_torque = carstate.steeringPressed

        desired_lane_width = lane_width_left if carstate.leftBlinker else lane_width_right
        lane_available = desired_lane_width >= self.config.lane_detection_width or not self.config.lane_detection
        nudgeless_ready = self.config.nudgeless and not manual_torque and lane_available and \
                          self.lane_change_wait_timer >= self.config.lane_change_delay

        if manual_torque:
          # Skip the delay so nudgeless logic doesn't immediately retrigger
          self.lane_change_wait_timer = self.config.lane_change_delay

        torque_applied = manual_torque or nudgeless_ready

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        if not one_blinker or below_lane_change_speed or self.lane_change_completed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif torque_applied and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_completed = self.config.one_lane_change
          self.lane_change_wait_timer = 0.0

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.lane_change_completed &= one_blinker
    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none
