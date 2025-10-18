#!/usr/bin/env python3
import math

import numpy as np

from common.params import Params


CRUISING_SPEED = 5.0  # m/s
TARGET_LAT_A = 2.0  # m/s^2
PARAM_REFRESH_SECONDS = 1.0
CURVE_SENSITIVITY_DEFAULT = 100
CURVE_SENSITIVITY_MIN = 50
CURVE_SENSITIVITY_MAX = 200
TURN_AGGRESSIVENESS_DEFAULT = 100
TURN_AGGRESSIVENESS_MIN = 50
TURN_AGGRESSIVENESS_MAX = 200


class VisionTurnSpeedController:
  def __init__(self):
    self._params = Params()
    self._refresh_counter = 0.0

    self.enabled = False
    self.curve_sensitivity = CURVE_SENSITIVITY_DEFAULT / 100.0
    self.turn_aggressiveness = TURN_AGGRESSIVENESS_DEFAULT / 100.0

    self.last_target_speed = 0.0
    self.last_curvature = 0.0

    self._read_params(force=True)

  def update(self, v_cruise, v_ego, enabled, left_blinker, right_blinker, model_v2, dt):
    self._refresh_counter -= dt
    if self._refresh_counter <= 0.0:
      self._read_params()
      self._refresh_counter = PARAM_REFRESH_SECONDS

    target_speed = v_cruise
    self.last_target_speed = v_cruise
    self.last_curvature = 0.0

    if not self.enabled or not enabled or model_v2 is None:
      return target_speed

    if v_ego <= CRUISING_SPEED:
      return target_speed

    if left_blinker or right_blinker:
      return target_speed

    curvature = self._compute_curvature(model_v2, v_ego)
    if curvature is None:
      return target_speed

    curvature_mag = abs(curvature)
    self.last_curvature = curvature
    if curvature_mag < 1e-6:
      return target_speed

    # Step out when the predicted curve can be taken comfortably at the current speed.
    if math.sqrt(1.0 / curvature_mag) >= v_ego:
      return target_speed

    denominator = curvature_mag * max(self.curve_sensitivity, 1e-3)
    numerator = TARGET_LAT_A * max(self.turn_aggressiveness, 1e-3)
    vtsc_speed = math.sqrt(numerator / denominator)
    vtsc_speed = max(CRUISING_SPEED, vtsc_speed)

    target_speed = min(target_speed, vtsc_speed)
    self.last_target_speed = target_speed
    return target_speed

  def _compute_curvature(self, model_v2, v_ego):
    orientation_rate = np.asarray(model_v2.orientationRate.z)
    velocity = np.asarray(model_v2.velocity.x)

    if orientation_rate.size == 0 or velocity.size == 0:
      return None

    lat_accels = orientation_rate * velocity
    if lat_accels.size == 0 or np.all(np.isnan(lat_accels)):
      return None

    max_idx = int(np.nanargmax(np.abs(lat_accels)))
    max_lat_accel = float(lat_accels[max_idx])
    v = max(v_ego, 1.0)
    return max_lat_accel / (v * v)

  def _read_params(self, force=False):
    enabled = self._params.get_bool("VisionTurnControl")
    if force or enabled != self.enabled:
      self.enabled = enabled

    curve_sensitivity = self._get_percent_param("CurveSensitivity",
                                                CURVE_SENSITIVITY_DEFAULT,
                                                CURVE_SENSITIVITY_MIN,
                                                CURVE_SENSITIVITY_MAX)
    turn_aggressiveness = self._get_percent_param("TurnAggressiveness",
                                                  TURN_AGGRESSIVENESS_DEFAULT,
                                                  TURN_AGGRESSIVENESS_MIN,
                                                  TURN_AGGRESSIVENESS_MAX)

    self.curve_sensitivity = curve_sensitivity / 100.0
    self.turn_aggressiveness = turn_aggressiveness / 100.0

  def _get_percent_param(self, key, default, min_value, max_value):
    value = self._params.get(key)
    if value is None:
      self._params.put(key, str(default))
      return default

    if isinstance(value, bytes):
      value = value.decode()
    value = value.strip()

    valid = True
    try:
      parsed = int(value)
    except ValueError:
      parsed = default
      valid = False

    clipped = int(np.clip(parsed, min_value, max_value))
    if not valid or clipped != parsed:
      self._params.put(key, str(clipped))

    return clipped
