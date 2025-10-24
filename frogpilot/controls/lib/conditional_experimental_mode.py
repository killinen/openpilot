#!/usr/bin/env python3
from enum import IntEnum

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, THRESHOLD

class CEStatus(IntEnum):
  OFF = 0               # Off
  USER_DISABLED = 1     # "Experimental Mode" disabled by user
  USER_OVERRIDDEN = 2   # "Experimental Mode" enabled by user
  SPEED = 3             # Speed condition
  SIGNAL = 4            # Turn signal condition
  CURVATURE = 5         # Road curvature condition
  LEAD = 6              # Slower lead vehicle condition
  STOP_LIGHT = 7        # Stop light or sign condition
  SPEED_LIMIT = 8       # Speed limit controller condition

class ConditionalExperimentalMode:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.curvature_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.slow_lead_filter = FirstOrderFilter(0, 1, DT_MDL)
    self.stop_light_filter = FirstOrderFilter(0, 0.5, DT_MDL)

    self.curve_detected = False
    self.experimental_mode = False
    self.stop_light_detected = False

  def update(self, v_ego, params_memory, sm, frogpilot_toggles):
    if frogpilot_toggles.experimental_mode_via_press:
      self.status_value = params_memory.get("CEStatus")
    else:
      self.status_value = CEStatus.OFF

    if self.status_value not in (CEStatus.USER_DISABLED, CEStatus.USER_OVERRIDDEN) and not sm["carState"].standstill:
      self.update_conditions(v_ego, sm, frogpilot_toggles)
      self.experimental_mode = self.check_conditions(v_ego, sm, frogpilot_toggles)
      params_memory.put("CEStatus", self.status_value if self.experimental_mode else CEStatus.OFF)
    else:
      self.experimental_mode = self.status_value == CEStatus.USER_OVERRIDDEN or sm["carState"].standstill and self.experimental_mode and self.frogpilot_planner.model_stopped
      self.experimental_mode &= self.status_value != CEStatus.USER_DISABLED

      self.stop_light_detected &= self.status_value not in (CEStatus.USER_DISABLED, CEStatus.USER_OVERRIDDEN)
      self.stop_light_filter.x = 0

  def check_conditions(self, v_ego, sm, frogpilot_toggles):
    below_speed = frogpilot_toggles.conditional_limit > v_ego >= 1 and not self.frogpilot_planner.frogpilot_following.following_lead
    below_speed_with_lead = frogpilot_toggles.conditional_limit_lead > v_ego >= 1 and self.frogpilot_planner.frogpilot_following.following_lead
    if below_speed or below_speed_with_lead:
      self.status_value = CEStatus.SPEED
      return True

    desired_lane = self.frogpilot_planner.lane_width_left if sm["carState"].leftBlinker else self.frogpilot_planner.lane_width_right
    lane_available = desired_lane >= frogpilot_toggles.lane_detection_width or not frogpilot_toggles.conditional_signal_lane_detection
    if v_ego < frogpilot_toggles.conditional_signal and (sm["carState"].leftBlinker or sm["carState"].rightBlinker) and not lane_available:
      self.status_value = CEStatus.SIGNAL
      return True

    if frogpilot_toggles.conditional_curves and self.curve_detected and (frogpilot_toggles.conditional_curves_lead or not self.frogpilot_planner.frogpilot_following.following_lead):
      self.status_value = CEStatus.CURVATURE
      return True

    if frogpilot_toggles.conditional_lead and self.slow_lead_detected:
      self.status_value = CEStatus.LEAD
      return True

    if frogpilot_toggles.conditional_model_stop_time != 0 and self.stop_light_detected:
      self.status_value = CEStatus.STOP_LIGHT
      return True

    if self.frogpilot_planner.frogpilot_vcruise.slc.experimental_mode:
      self.status_value = CEStatus.SPEED_LIMIT
      return True

    return False

  def update_conditions(self, v_ego, sm, frogpilot_toggles):
    self.curve_detection(v_ego, frogpilot_toggles)
    self.slow_lead(frogpilot_toggles)
    self.stop_sign_and_light(v_ego, sm, frogpilot_toggles.conditional_model_stop_time)

  def curve_detection(self, v_ego, frogpilot_toggles):
    curve_detected = self.frogpilot_planner.road_curvature_detected or self.frogpilot_planner.driving_in_curve

    self.curvature_filter.update(curve_detected)
    self.curve_detected = self.curvature_filter.x >= THRESHOLD and v_ego > CRUISING_SPEED

  def slow_lead(self, frogpilot_toggles):
    if self.frogpilot_planner.tracking_lead:
      slower_lead = frogpilot_toggles.conditional_slower_lead and self.frogpilot_planner.frogpilot_following.slower_lead
      stopped_lead = frogpilot_toggles.conditional_stopped_lead and self.frogpilot_planner.lead_one.vLead < 1

      self.slow_lead_filter.update(slower_lead or stopped_lead)
      self.slow_lead_detected = self.slow_lead_filter.x >= THRESHOLD
    else:
      self.slow_lead_filter.x = 0
      self.slow_lead_detected = False

  def stop_sign_and_light(self, v_ego, sm, model_time):
    if not sm["frogpilotCarState"].trafficModeEnabled:
      model_stopping = self.frogpilot_planner.model_length < v_ego * model_time

      self.stop_light_filter.update(self.frogpilot_planner.model_stopped or model_stopping)
      self.stop_light_detected = self.stop_light_filter.x >= THRESHOLD and not self.frogpilot_planner.tracking_lead
    else:
      self.stop_light_filter.x = 0
      self.stop_light_detected = False
