import numpy as np
from abc import abstractmethod, ABC

MIN_LATERAL_CONTROL_SPEED = 0.3  # m/s


class LatControl(ABC):
  def __init__(self, CP, CI, dt):
    self.dt = dt
    self.sat_limit = CP.steerLimitTimer
    self.sat_time = 0.
    self.sat_check_min_speed = 10.

    # Most cars use [-1, 1]. Extra headroom can extend the limit while keeping
    # the nominal torque scale (and therefore ordinary-range gains) unchanged.
    self.steer_max = CI.get_steer_max(CP)

  @abstractmethod
  def update(self, active: bool, CS, VM, params, steer_limited_by_safety: bool, desired_curvature: float, curvature_limited: bool, lat_delay: float, llk, model_data, frogpilot_toggles):
    pass

  def reset(self):
    self.sat_time = 0.

  def _check_saturation(self, saturated, CS, steer_limited_by_safety, curvature_limited):
    # Saturated only if control output is not being limited by car torque/angle rate limits
    if (saturated or curvature_limited) and CS.vEgo > self.sat_check_min_speed and not steer_limited_by_safety and not CS.steeringPressed:
      self.sat_time += self.dt
    else:
      self.sat_time -= self.dt
    self.sat_time = np.clip(self.sat_time, 0.0, self.sat_limit)
    return self.sat_time > (self.sat_limit - 1e-3)
