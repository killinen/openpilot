from collections import deque

from cereal import car
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.i30.values import DBC, Buttons, CarControllerParams

PREV_BUTTON_SAMPLES = 8

GearShifter = car.CarState.GearShifter

I30_GEAR_RATIO_TOLERANCES = (
  (130.6, 10.0),
  (105.7, 8.0),
  (92.1, 6.0),
)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

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

  def update(self, cp, cp_cam):
    return self.update_i30(cp, cp_cam)

  def update_i30(self, cp, cp_cam):
    ret = car.CarState.new_message()

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

    # Gear ratio calculation using engine RPM and vehicle speed
    ret.clutchPressed = False
    if ret.vEgo > 0.3 and ret.engineRpm > 500:
      rpm_velo_ratio = ret.engineRpm / ret.vEgo
      in_gear = any(abs(rpm_velo_ratio - ratio) < tol for ratio, tol in I30_GEAR_RATIO_TOLERANCES)
      ret.clutchPressed = not in_gear

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
    ret.steeringTorqueOut = cp.vl["VSM2"]["CR_Mdps_OutTq"]
    ret.steeringTorqueEps = cp_cam.vl["STEERING_STATUS"]['STEERING_TORQUE']
    ret.steeringAngleDegError = 0.0
    ret.steeringAngleDegDivergence = 0.0

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
          ret.steeringAngleDegDivergence = self.i30_max_error - self.i30_min_error

        ret.steeringAngleDegError = angle_error

    # emulate driver steering torque - allows lane change assist on blinker hold
    ret.steeringPressed = ret.gasPressed    # i30 with SSC doesn't have separate torque sensor, so lightly pressing the gas indicates driver intention to change lane

    ret.cruiseState.available = cp.vl["EMS6"]['CRUISE_LAMP_M'] != 0
    ret.cruiseState.enabled = bool(cp.vl["EMS6"]['CRUISE_LAMP_S'])
    ret.cruiseState.standstill = False
    ret.cruiseState.speed = 0

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["EMS_DCT2"]['BRAKE_ACT'] == 2
    ret.gas = cp.vl["EMS_DCT1"]['PV_AV_CAN']
    ret.gearShifter = GearShifter.reverse if cp.vl["CLU2"]['CF_Clu_SwiGearR'] else GearShifter.drive  # Force D-gear otherwise because my car is manual
    # TODO: You should make something up for these when have the time
    self.brake_error = False
    self.park_brake = False

    self.prev_cruise_buttons = self.cruise_buttons[-1]  # Get the last button pressed
    self.cruise_buttons.extend(cp.vl_all["CLU1"]["CF_Clu_CruiseSwState"])

    return ret

  @staticmethod
  def get_can_parser(CP):
    return CarState.get_can_parser_i30(CP)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("STEERING_TORQUE", "STEERING_STATUS", 0),
      ("STEERING_ANGLE", "STEERING_STATUS", 0),
    ]
    checks = [
      ("STEERING_STATUS", 20)    # Checks if SSC is connected
    ]
    if CP.enableGasInterceptor:
      signals += [("INTERCEPTOR_GAS", "GAS_SENSOR"),
                  ("INTERCEPTOR_GAS2", "GAS_SENSOR")]
      checks.append(("GAS_SENSOR", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1)

  # CAR CAN parser for I30
  @staticmethod
  def get_can_parser_i30(CP):
    signals = [
      # sig_name, sig_address, default
      ("N", "EMS1"),
      ("WHEEL_FL", "TCS5"),
      ("WHEEL_FR", "TCS5"),
      ("WHEEL_RL", "TCS5"),
      ("WHEEL_RR", "TCS5"),
      ("YAW_RATE", "ESP2"),
      ("CF_Clu_DrvSeatBeltSw", "CLU2"),
      ("CF_Clu_DrvDrSw", "CLU2"),          # Driver Door
      ("CF_Clu_AstDrSw", "CLU2"),          # Passenger door
      ("CF_Clu_TurnSigLh", "CLU2"),
      ("CF_Clu_TurnSigRh", "CLU2"),
      ("CF_Clu_SwiGearR", "CLU2"),
      ("CF_Clu_CruiseSwState", "CLU1"),
      ("CRUISE_LAMP_M", "EMS6"),
      ("CRUISE_LAMP_S", "EMS6"),
      ("BRAKE_ACT", "EMS2"),
      ("BRAKE_ACT", "EMS_DCT2"),
      ("PV_AV_CAN", "EMS_DCT1"),
      ("CF_Ems_AclAct", "EMS6"),
      ("CR_Mdps_StrTq", "VSM2"),
      ("CR_Mdps_OutTq", "VSM2"),
      ("SAS_Angle", "SAS1"),
      ("SAS_Speed", "SAS1"),
    ]

    checks = [
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

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)
