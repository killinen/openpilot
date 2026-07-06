from math import fabs

from cereal import car
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.i30.values import CarControllerParams
from openpilot.selfdrive.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_steer_feedforward_sigmoid(desired_angle, v_ego):
    v_ego = min(v_ego, 25)

    desired_angle *= 0.0205
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.10006696 * sigmoid * (v_ego + 3.12485927)

  def get_steer_feedforward_function(self):
    return self.get_steer_feedforward_sigmoid

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles):
    ret.carName = "motorhome"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.motorhome)]
    ret.radarUnavailable = True

    ret.openpilotLongitudinalControl = False
    ret.experimentalLongitudinalAvailable = False
    ret.pcmCruise = True
    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.4

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[5.5, 30.], [5.5, 30.]]
    ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.0004, 0.0004], [0.10, 0.12]]
    ret.lateralTuning.pid.kf = 1.

    return ret

  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)
    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise)
    ret.events = events.to_msg()

    return ret, fp_ret
