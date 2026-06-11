from collections import deque

from cereal import car, custom
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.i30.values import DBC, Buttons, CarControllerParams, i30_uses_trqi_steering

PREV_BUTTON_SAMPLES = 8

GearShifter = car.CarState.GearShifter

I30_GEAR_RATIO_WINDOWS = (
  (130.6, 10.0),
  (105.7, 8.0),
  (92.1, 6.0),
)
I30_MIN_CLUTCH_SPEED = 2.0
I30_MIN_CLUTCH_RPM = 700.0
I30_CLUTCH_PRESS_TOL_MULTIPLIER = 2.0
I30_CLUTCH_RELEASE_TOL_MULTIPLIER = 0.8
I30_CLUTCH_PRESS_FRAMES = 2
I30_CLUTCH_RELEASE_FRAMES = 4


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)

    self.cruise_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.prev_cruise_buttons = Buttons.NONE

    self.params = CarControllerParams(CP)

    self.i30_angle_offset_needed = True
    self.i30_angle_offset = 0.0
    self.i30_angle_aligned = False
    self.i30_min_error = 0.0
    self.i30_max_error = 0.0
    self.i30_ssc_angle_initialized = False
    self.i30_ssc_angle_last = 0.0
    self.i30_ssc_angle_unwrapped = 0.0

    self.brake_error = False
    self.park_brake = False
    self.i30_clutch_pressed = False
    self.i30_clutch_press_samples = 0
    self.i30_clutch_release_samples = 0
    self.steering_torque_out = 0.0
    self.trqi_disengage_error = False
    self.trqi_non_disengage_error = False
    self.trqi_limit = False
    self.trqi_host_command_limited = False

  def update(self, cp, cp_cam, frogpilot_toggles):
    return self.update_i30(cp, cp_cam)

  def update_clutch_state(self, v_ego, engine_rpm):
    if v_ego < I30_MIN_CLUTCH_SPEED or engine_rpm < I30_MIN_CLUTCH_RPM:
      self.i30_clutch_press_samples = 0
      self.i30_clutch_release_samples = 0
      return self.i30_clutch_pressed

    rpm_velo_ratio = engine_rpm / v_ego
    closest_ratio, base_tol = min(I30_GEAR_RATIO_WINDOWS, key=lambda ratio_tol: abs(rpm_velo_ratio - ratio_tol[0]))
    ratio_error = abs(rpm_velo_ratio - closest_ratio)
    press_tol = base_tol * I30_CLUTCH_PRESS_TOL_MULTIPLIER
    release_tol = base_tol * I30_CLUTCH_RELEASE_TOL_MULTIPLIER

    if self.i30_clutch_pressed:
      self.i30_clutch_press_samples = 0
      if ratio_error < release_tol:
        self.i30_clutch_release_samples += 1
        if self.i30_clutch_release_samples >= I30_CLUTCH_RELEASE_FRAMES:
          self.i30_clutch_pressed = False
          self.i30_clutch_release_samples = 0
      else:
        self.i30_clutch_release_samples = 0
    else:
      self.i30_clutch_release_samples = 0
      if ratio_error > press_tol:
        self.i30_clutch_press_samples += 1
        if self.i30_clutch_press_samples >= I30_CLUTCH_PRESS_FRAMES:
          self.i30_clutch_pressed = True
          self.i30_clutch_press_samples = 0
      else:
        self.i30_clutch_press_samples = 0

    return self.i30_clutch_pressed

  def update_i30(self, cp, cp_cam):
    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()

    ret.doorOpen = any([cp.vl["CLU2"]['CF_Clu_DrvDrSw'], cp.vl["CLU2"]['CF_Clu_AstDrSw']])

    ret.seatbeltUnlatched = cp.vl["CLU2"]['CF_Clu_DrvSeatBeltSw'] == 1

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["TCS5"]["WHEEL_FL"],
      cp.vl["TCS5"]["WHEEL_FR"],
      cp.vl["TCS5"]["WHEEL_RL"],
      cp.vl["TCS5"]["WHEEL_RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.engineRpm = cp.vl["EMS1"]["N"]

    # Estimate clutch state from the nearest learned RPM/speed ratio window.
    ret.clutchPressed = self.update_clutch_state(ret.vEgo, ret.engineRpm)

    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = cp.vl["SAS1"]['SAS_Angle']
    ret.steeringRateDeg = cp.vl["SAS1"]['SAS_Speed']
    ret.yawRate = cp.vl["ESP2"]['YAW_RATE']
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["CLU2"]['CF_Clu_TurnSigLh'],
                                                                      cp.vl["CLU2"]['CF_Clu_TurnSigRh'])

    if self.CP.enableGasInterceptor:
      ret.gas = (cp_cam.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp_cam.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      # TODO: tune this threshold --> 805 is a good start
      ret.gasPressed = ret.gas > 5
    else:
      ret.gasPressed = cp.vl["EMS6"]['CF_Ems_AclAct'] > 0.05

    ret.steeringTorque = cp.vl["VSM2"]["CR_Mdps_StrTq"]
    self.steering_torque_out = cp.vl["VSM2"]["CR_Mdps_OutTq"]
    fp_ret.steeringTorqueOut = self.steering_torque_out

    if i30_uses_trqi_steering():
      # TRQI mode replaces the old STEERING_STATUS heartbeat with TRQI_IOStatus.
      # The standalone board does not publish the old angle/torque feedback, so
      # disable the SSC-specific alignment path when this mode is selected.
      ret.steeringTorqueEps = 0.0
      self.trqi_disengage_error = bool(cp_cam.vl["TRQI_FaultStatus"]["Disengage_Error"])
      self.trqi_non_disengage_error = bool(cp_cam.vl["TRQI_FaultStatus"]["Non_Disengage_Error"])
      self.trqi_limit = bool(cp_cam.vl["TRQI_FaultStatus"]["Any_TRQI_Limit"])
      self.trqi_host_command_limited = bool(cp_cam.vl["TRQI_FaultStatus"]["Host_Command_Limited"])
      self.i30_angle_offset_needed = True
      self.i30_angle_aligned = False
      self.i30_ssc_angle_initialized = False
    else:
      self.trqi_disengage_error = False
      self.trqi_non_disengage_error = False
      self.trqi_limit = False
      self.trqi_host_command_limited = False
      ret.steeringTorqueEps = cp_cam.vl["STEERING_STATUS"]['STEERING_TORQUE']

      ssc_can_valid = bool(getattr(cp_cam, "can_valid", False))
      if not ssc_can_valid:
        self.i30_angle_offset_needed = True
        self.i30_angle_aligned = False
        self.i30_ssc_angle_initialized = False
      else:
        ssc_angle = cp_cam.vl["STEERING_STATUS"]["STEERING_ANGLE"] * (16.0 / 26.0)  # convert SSC gear ratio (16/26)

        # Unwrap SSC angle to avoid false divergence on wrap/reset.
        if not self.i30_ssc_angle_initialized:
          self.i30_ssc_angle_initialized = True
          self.i30_ssc_angle_last = ssc_angle
          self.i30_ssc_angle_unwrapped = ssc_angle
        else:
          delta = ssc_angle - self.i30_ssc_angle_last
          if delta > 180.0:
            delta -= 360.0
          elif delta < -180.0:
            delta += 360.0
          self.i30_ssc_angle_unwrapped += delta
          self.i30_ssc_angle_last = ssc_angle

        steering_status_angle = self.i30_ssc_angle_unwrapped
        if self.i30_angle_offset_needed:
          self.i30_angle_offset = steering_status_angle - ret.steeringAngleDeg
          self.i30_angle_offset_needed = False
          self.i30_angle_aligned = False
        else:
          ssc_aligned_angle = steering_status_angle - self.i30_angle_offset
          angle_error = ssc_aligned_angle - ret.steeringAngleDeg

          if not self.i30_angle_aligned and abs(angle_error) < 0.1:
            self.i30_angle_aligned = True
          self.i30_min_error = angle_error
          self.i30_max_error = angle_error

          if self.i30_angle_aligned:
            self.i30_min_error = min(self.i30_min_error, angle_error)
            self.i30_max_error = max(self.i30_max_error, angle_error)
            fp_ret.steeringAngleDegDivergence = self.i30_max_error - self.i30_min_error

          fp_ret.steeringAngleDegError = angle_error
    # emulate driver steering torque - allows lane change assist on blinker hold
    ret.steeringPressed = ret.gasPressed    # i30 with SSC doesn't have separate torque sensor, so lightly pressing the gas indicates driver intention to change lane

    # Allow openpilot set speed even when stock cruise main is off.
    ret.cruiseState.available = self.CP.openpilotLongitudinalControl or (cp.vl["EMS6"]['CRUISE_LAMP_M'] != 0)
    ret.cruiseState.enabled = bool(cp.vl["EMS6"]['CRUISE_LAMP_S'])
    ret.cruiseState.standstill = False
    ret.cruiseState.speed = 0

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["EMS_DCT2"]['BRAKE_ACT'] == 2
    ret.gas = cp.vl["EMS_DCT1"]['PV_AV_CAN']
    ret.gearShifter = GearShifter.reverse if cp.vl["CLU2"]['CF_Clu_SwiGearR'] else GearShifter.drive  # Force D-gear otherwise because my car is manual
    ret.parkingBrake = self.park_brake
    # TODO: You should make something up for these when have the time
    self.brake_error = False
    self.park_brake = False

    self.prev_cruise_buttons = self.cruise_buttons[-1]  # Get the last button pressed
    self.cruise_buttons.extend(cp.vl_all["CLU1"]["CF_Clu_CruiseSwState"])
    self.main_buttons.extend(cp.vl_all["CLU1"]["CF_Clu_CruiseSwMain"])

    return ret, fp_ret

  @staticmethod
  def get_can_parser(CP, FPCP):
    return CarState.get_can_parser_i30(CP)

  @staticmethod
  def get_cam_can_parser(CP, FPCP):
    messages = []
    if i30_uses_trqi_steering():
      # Use TRQI_IOStatus as the actuator-side heartbeat when the standalone TRQI
      # board is in charge of steering. This keeps cp_cam valid without requiring
      # the old STEERING_STATUS message to still be present on bus 1.
      messages.append(("TRQI_IOStatus", 10))
      messages.append(("TRQI_FaultStatus", 10))
    else:
      messages.append(("STEERING_STATUS", 20))  # Checks if SSC is connected
    if CP.enableGasInterceptor:
      messages.append(("GAS_SENSOR", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 1)

  # CAR CAN parser for I30
  @staticmethod
  def get_can_parser_i30(CP):
    messages = [
      ("EMS1", 20),
      ("EMS_DCT2", 20),  # True interval 10 ms
      ("VSM2", 20),      # True interval 10 ms
      ("TCS5", 20),      # True interval 20 ms
      ("SAS1", 20),      # True interval 10 ms
      ("EMS2", 20),      # True interval ? ms
      ("EMS6", 20),      # True interval ? ms
      ("EMS_DCT1", 20),  # True interval ? ms
      ("ESP2", 20),      # True interval ? ms
      ("CLU1", 20),      # True interval ? ms
      ("CLU2", 10),      # True interval ? ms
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)
