from math import fabs

from cereal import car
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.car.i30.values import Buttons, CarControllerParams

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
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs, frogpilot_toggles):
    ret.carName = "i30"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.hyundaiCommunity, 4)]
    ret.radarUnavailable = True

    ret.openpilotLongitudinalControl = True
    ret.experimentalLongitudinalAvailable = False
    ret.pcmCruise = False
    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.4

    ret.stoppingControl = True
    ret.vEgoStopping = 1.0

    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kpV = [0.1]
    ret.longitudinalTuning.kiV = [0.0]
    ret.stopAccel = 0.0
    ret.longitudinalActuatorDelay = 0.2

    ret.enableGasInterceptor = True

    if ret.enableGasInterceptor:
      ret.longitudinalTuning.kpBP = [0., 15., 30.]
      ret.longitudinalTuning.kiBP = [0., 15., 30.]
      ret.longitudinalTuning.kpV = [0.1, 0.2, 0.3]
      ret.longitudinalTuning.kiV = [0.02, 0.025, 0.03]

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    return ret

  def _update(self, c, frogpilot_toggles):
    ret, fp_ret = self.CS.update(self.cp, self.cp_cam, frogpilot_toggles)

    if self.CP.openpilotLongitudinalControl:
      ret.buttonEvents = create_button_events(self.CS.cruise_buttons[-1], self.CS.prev_cruise_buttons, BUTTONS_DICT)

    allow_enable = any(btn in ENABLE_BUTTONS for btn in self.CS.cruise_buttons) or any(self.CS.main_buttons)
    events = self.create_common_events(ret, pcm_enable=self.CS.CP.pcmCruise, allow_enable=allow_enable)

    # Allow engagement with openpilot long control even if stock cruise is not available
    if self.CP.openpilotLongitudinalControl and EventName.wrongCarMode in events.names:
      events.events.remove(EventName.wrongCarMode)

    if ret.clutchPressed:
      events.add(EventName.pedalPressed)

    if self.CS.brake_error:
      events.add(EventName.brakeUnavailable)

    if self.CS.trqi_disengage_error:
      events.add(EventName.trqiDisengageError)
    if self.CS.trqi_non_disengage_error:
      events.add(EventName.trqiNonDisengageError)
    if self.CS.trqi_limit:
      events.add(EventName.trqiLimit)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    return ret, fp_ret
