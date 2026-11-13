#!/usr/bin/env python3
import json
import numpy as np

from openpilot.common.realtime import DT_MDL

from openpilot.frogpilot.common.frogpilot_variables import CRUISING_SPEED, DEFAULT_LATERAL_ACCELERATION, PLANNER_TIME, params

CALIBRATION_PROGRESS_THRESHOLD = 10 / DT_MDL
MAX_CURVATURE = 0.1
MIN_CURVATURE = 0.001
PERCENTILE = 90
ROUNDING_PRECISION = 5
STEP = 0.001

# Braking envelope for upcoming curves
CURVE_DECEL = 1.0         # m/s^2, comfortable deceleration while approaching a curve
CURVE_DECEL_MIN = 0.5     # m/s^2, lowest deceleration used with gentle deceleration profiles
MIN_CURVATURE_EPS = 1e-5  # 1/m, keeps straight-road speed limits finite
MIN_PATH_SPEED = 1.0      # m/s, lower bound on model path speed when converting yaw rate to curvature
REACTION_TIME = 0.75      # s, planner and actuator lag before braking takes effect
TARGET_FALL_RATE = 2.0    # m/s^2, fastest the target may drop below v_ego (rejects single-frame model spikes)
TARGET_RISE_RATE = 1.0    # m/s^2, how quickly the target releases when leaving a curve
MAX_TARGET = 70.0         # m/s, published target when no curve limits speed

def calculate_curve_speed_limit(orientation_rate, velocity, position, v_ego, lateral_acceleration, decel):
  """Highest speed allowed now so that the car can slow at `decel` and reach every point on
  the predicted path at or below the speed that gives `lateral_acceleration` there."""
  orientation_rate = np.abs(np.asarray(orientation_rate, dtype=float))
  velocity = np.asarray(velocity, dtype=float)
  position = np.asarray(position, dtype=float)
  if not (len(orientation_rate) == len(velocity) == len(position) > 0):
    return float("inf")

  # The path curvature is yaw rate over the model's own predicted speed; using v_ego would
  # underestimate it whenever the model already expects to slow down for the curve.
  curvature = orientation_rate / np.maximum(velocity, MIN_PATH_SPEED)
  curve_speed_squared = lateral_acceleration / np.maximum(curvature, MIN_CURVATURE_EPS)

  braking_distance = np.maximum(position - v_ego * REACTION_TIME, 0)
  return float(np.sqrt(np.min(curve_speed_squared + 2 * decel * braking_distance)))

class CurveSpeedController:
  def __init__(self, FrogPilotVCruise):
    self.frogpilot_planner = FrogPilotVCruise.frogpilot_planner

    self.enable_training = False
    self.target_set = False

    self.training_timer = 0

    self.curvature_data = json.loads(params.get("CurvatureData") or "{}")

    self.required_curvatures = [str(round(road_curvature, ROUNDING_PRECISION)) for road_curvature in np.arange(MIN_CURVATURE, MAX_CURVATURE + STEP, STEP)]

    self.update_lateral_acceleration()

  def log_data(self, v_ego, sm):
    self.enable_training = v_ego > CRUISING_SPEED
    self.enable_training &= not self.frogpilot_planner.tracking_lead
    self.enable_training &= not sm["carControl"].longActive

    if self.enable_training:
      self.training_timer += DT_MDL

      if self.training_timer >= PLANNER_TIME and self.frogpilot_planner.driving_in_curve and not (sm["carState"].leftBlinker or sm["carState"].rightBlinker):
        lateral_acceleration = abs(self.frogpilot_planner.lateral_acceleration)
        road_curvature = abs(round(self.frogpilot_planner.road_curvature, ROUNDING_PRECISION))

        key = str(road_curvature)
        if key in self.curvature_data:
          data = self.curvature_data[key]

          average = data["average"]
          count = data["count"]

          self.curvature_data[key] = {
            "average": ((average * count) + lateral_acceleration) / (count + 1),
            "count": count + 1
          }
        else:
          self.curvature_data[key] = {
            "average": lateral_acceleration,
            "count": 1
          }

        self.update_lateral_acceleration()
      else:
        self.enable_training = False

    elif self.training_timer >= PLANNER_TIME:
      progress = 0.0
      for key in self.required_curvatures:
        if key in self.curvature_data:
          progress += min(self.curvature_data[key]["count"] / CALIBRATION_PROGRESS_THRESHOLD, 1.0)

      params.put_float_nonblocking("CalibrationProgress", (progress / len(self.required_curvatures)) * 100)
      params.put_nonblocking("CurvatureData", json.dumps(self.curvature_data))

      self.enable_training = False

      self.training_timer = 0

    else:
      self.enable_training = False

      self.training_timer = 0

  def update_lateral_acceleration(self):
    if self.curvature_data:
      all_samples = [data["average"] for data in self.curvature_data.values()]
      self.lateral_acceleration = float(np.percentile(all_samples, PERCENTILE))
    else:
      self.lateral_acceleration = DEFAULT_LATERAL_ACCELERATION

    params.put_float_nonblocking("CalibratedLateralAcceleration", self.lateral_acceleration)

  def update_target(self, v_ego, sm, frogpilot_toggles):
    lateral_acceleration = frogpilot_toggles.curve_speed_lateral_acceleration
    if self.frogpilot_planner.frogpilot_weather.weather_id != 0:
      lateral_acceleration -= lateral_acceleration * self.frogpilot_planner.frogpilot_weather.reduce_lateral_acceleration

    # Brake no harder than the active deceleration profile lets the planner follow.
    min_accel = self.frogpilot_planner.frogpilot_acceleration.min_accel
    decel = float(np.clip(-min_accel, CURVE_DECEL_MIN, CURVE_DECEL)) if min_accel < 0 else CURVE_DECEL

    # Lane changes bend the predicted path without a curve in the road, so release gently instead.
    if sm["carState"].leftBlinker or sm["carState"].rightBlinker:
      speed_limit = MAX_TARGET
    else:
      model = sm["modelV2"]
      speed_limit = calculate_curve_speed_limit(model.orientationRate.z, model.velocity.x, model.position.x,
                                                v_ego, lateral_acceleration, decel)
    speed_limit = min(speed_limit, MAX_TARGET)

    if not self.target_set:
      self.target_set = True
      self.target = max(speed_limit, v_ego)

    # Rate-limit around the previous target so single-frame model spikes cannot snap the cap down
    # and the release after a curve is gradual. The limits apply only on the far side of v_ego:
    # a curve that first requires slowing starts the target at v_ego instead of a stale higher
    # value, and a path that allows v_ego never holds the target below it.
    target = float(np.clip(speed_limit, self.target - TARGET_FALL_RATE * DT_MDL, self.target + TARGET_RISE_RATE * DT_MDL))
    target = float(np.clip(target, min(speed_limit, v_ego), max(speed_limit, v_ego)))
    self.target = max(target, CRUISING_SPEED)
