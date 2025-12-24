#!/usr/bin/env python3
from math import fabs

from cereal import car
from selfdrive.car import create_button_enable_events, create_button_event, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.i30.values import Buttons, CarControllerParams

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
ENABLE_BUTTONS = (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel}


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
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[], disable_radar=False):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "i30"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.i30, 0)]
    ret.radarOffCan = True

    ret.openpilotLongitudinalControl = True
    ret.pcmCruise = False
    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.4
    tire_stiffness_factor = 0.385

    ret.stoppingControl = True
    ret.vEgoStopping = 1.0

    ret.longitudinalTuning.kpV = [0.1]
    ret.longitudinalTuning.kiV = [0.0]
    ret.stopAccel = 0.0
    ret.longitudinalActuatorDelayUpperBound = 0.2

    ret.enableGasInterceptor = True
    ret.safetyConfigs[0].safetyParam = 4     # Detect 4 in panda safety code to use pedal stuff and op cruise w i30

    ret.mass = 1193
    ret.wheelbase = 2.650
    ret.steerRatio = 15.3

    if ret.enableGasInterceptor:
      ret.longitudinalTuning.kpBP = [0., 15., 30.]
      ret.longitudinalTuning.kiBP = [0., 15., 30.]
      ret.longitudinalTuning.kpV = [0.2, 0.4, 0.6]
      ret.longitudinalTuning.kiV = [0.05, 0.1, 0.15]

    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[5.5, 30.], [5.5, 30.]]
    ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.0004, 0.0004], [0.10, 0.12]]
    ret.lateralTuning.pid.kf = 1.

    ret.maxSteeringAngleDeg = 90
    ret.radarTimeStep = 0.05  # time delta between radar updates, 20Hz is very standard

    ret.centerToFront = ret.wheelbase * 0.4
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)

    # Allow engagement with openpilot long control even if stock cruise is not available
    if self.CP.openpilotLongitudinalControl and EventName.wrongCarMode in events.names:
      events.events.remove(EventName.wrongCarMode)

    if self.CS.brake_error:
      events.add(EventName.brakeUnavailable)

    if self.CS.CP.openpilotLongitudinalControl and self.CS.cruise_buttons[-1] != self.CS.prev_cruise_buttons:
      buttonEvents = [create_button_event(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT)]
      # Handle CF_Clu_CruiseSwState changing buttons mid-press
      if self.CS.cruise_buttons[-1] != 0 and self.CS.prev_cruise_buttons != 0:
        buttonEvents.append(create_button_event(0, self.CS.prev_cruise_buttons, BUTTONS_DICT))

      ret.buttonEvents = buttonEvents
      events.events.extend(create_button_enable_events(ret.buttonEvents))

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    return self.CC.update(c, self.CS)
