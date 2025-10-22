from collections import deque
import copy

from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.hyundai.values import DBC, FEATURES, HDA2_CAR, EV_CAR, HYBRID_CAR, Buttons, CarControllerParams, CAR
from selfdrive.car.interfaces import CarStateBase

PREV_BUTTON_SAMPLES = 8

GearShifter = car.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.cruise_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.prev_cruise_buttons = Buttons.NONE

    if CP.carFingerprint in HDA2_CAR:
      self.shifter_values = can_define.dv["ACCELERATOR"]["GEAR"]
    elif self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    elif CP.carFingerprint != CAR.I30:  # preferred and elect gear methods use same definition unless 2014 I30
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    self.brake_error = False
    self.park_brake = False
    self.buttons_counter = 0

    self.params = CarControllerParams(CP)

    if CP.carFingerprint == CAR.I30:
      self.i30_angle_offset_needed = True
      self.i30_angle_offset = 0.0
      self.i30_angle_aligned = False
      self.i30_min_error = 0.0
      self.i30_max_error = 0.0

  def update(self, cp, cp_cam):
    if self.CP.carFingerprint in HDA2_CAR:
      return self.update_hda2(cp, cp_cam)
    if self.CP.carFingerprint == CAR.I30:
      return self.update_i30(cp, cp_cam)

    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_THRESHOLD
    ret.steerFaultTemporary = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0

    # cruise state
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.available = cp.vl["TCS13"]["ACCEnable"] == 0
      ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.available = cp.vl["SCC11"]["MainMode_ACC"] == 1
      ret.cruiseState.enabled = cp.vl["SCC12"]["ACCMode"] != 0
      ret.cruiseState.standstill = cp.vl["SCC11"]["SCCInfoDisplay"] == 4.
      speed_conv = CV.MPH_TO_MS if cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] else CV.KPH_TO_MS
      ret.cruiseState.speed = cp.vl["SCC11"]["VSetDis"] * speed_conv

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverBraking"] != 0
    ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2 # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1

    if self.CP.carFingerprint in (HYBRID_CAR | EV_CAR):
      if self.CP.carFingerprint in HYBRID_CAR:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
    elif self.CP.carFingerprint in FEATURES["use_elect_gears"]:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if not self.CP.openpilotLongitudinalControl:
      if self.CP.carFingerprint in FEATURES["use_fca"]:
        ret.stockAeb = cp.vl["FCA11"]["FCA_CmdAct"] != 0
        ret.stockFcw = cp.vl["FCA11"]["CF_VSM_Warn"] == 2
      else:
        ret.stockAeb = cp.vl["SCC12"]["AEB_CmdAct"] != 0
        ret.stockFcw = cp.vl["SCC12"]["CF_VSM_Warn"] == 2

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    # save the entire LKAS11 and CLU11
    self.lkas11 = copy.copy(cp_cam.vl["LKAS11"])
    self.clu11 = copy.copy(cp.vl["CLU11"])
    self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    self.brake_error = cp.vl["TCS13"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])

    return ret

  def update_hda2(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.gas = cp.vl["ACCELERATOR"]["ACCELERATOR_PEDAL"] / 255.
    ret.gasPressed = ret.gas > 1e-3
    ret.brakePressed = cp.vl["BRAKE"]["BRAKE_PRESSED"] == 1

    ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR_OPEN"] == 1
    ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT_LATCHED"] == 0

    gear = cp.vl["ACCELERATOR"]["GEAR"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    # TODO: figure out positions
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_1"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_2"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_3"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_4"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]
    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_THRESHOLD

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["BLINKERS"]["LEFT_LAMP"],
                                                                      cp.vl["BLINKERS"]["RIGHT_LAMP"])

    ret.cruiseState.available = True
    ret.cruiseState.enabled = cp.vl["SCC1"]["CRUISE_ACTIVE"] == 1
    ret.cruiseState.standstill = cp.vl["CRUISE_INFO"]["CRUISE_STANDSTILL"] == 1

    speed_factor = CV.MPH_TO_MS if cp.vl["CLUSTER_INFO"]["DISTANCE_UNIT"] == 1 else CV.KPH_TO_MS
    ret.cruiseState.speed = cp.vl["CRUISE_INFO"]["SET_SPEED"] * speed_factor

    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons.extend(cp.vl_all["CRUISE_BUTTONS"]["CRUISE_BUTTONS"])
    self.main_buttons.extend(cp.vl_all["CRUISE_BUTTONS"]["ADAPTIVE_CRUISE_MAIN_BTN"])
    self.buttons_counter = cp.vl["CRUISE_BUTTONS"]["COUNTER"]

    self.cam_0x2a4 = copy.copy(cp_cam.vl["CAM_0x2a4"])

    return ret

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
    GEAR_RATIO = [130.6, 105.7, 92.1]
    ret.clutchPressed = False
    if ret.vEgo > 0.3 and ret.engineRpm > 500:
      rpm_velo_ratio = ret.engineRpm / ret.vEgo
      in_gear = any(abs(rpm_velo_ratio - r) < 6 for r in GEAR_RATIO)
      if not in_gear:
        ret.clutchPressed = True

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

    steering_status_angle = cp_cam.vl["STEERING_STATUS"]["STEERING_ANGLE"] * (16.0 / 26.0)  # convert SSC gear ratio (16/26)
    if self.i30_angle_offset_needed:
      self.i30_angle_offset = steering_status_angle - ret.steeringAngleDeg
      self.i30_angle_offset_needed = False
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
    ret.gearShifter = GearShifter.reverse if cp.vl["CLU2"]['CF_Clu_SwiGearR'] else GearShifter.drive	# Force D-gear otherwise because my car is manual
    # TODO: You should make something up for these when have the time
    self.brake_error = False
    self.park_brake = False

    # Old (working) cruise buttons
    # self.prev_cruise_buttons = self.cruise_buttons
    # self.cruise_buttons = cp.vl["CLU1"]["CF_Clu_CruiseSwState"]

    # New cruise buttons
    self.prev_cruise_buttons = self.cruise_buttons[-1]  # Get the last button pressed
    self.cruise_buttons.extend(cp.vl_all["CLU1"]["CF_Clu_CruiseSwState"])

    return ret

  @staticmethod
  def get_can_parser(CP):
    if CP.carFingerprint in HDA2_CAR:
      return CarState.get_can_parser_hda2(CP)
    if CP.carFingerprint == CAR.I30:
      return CarState.get_can_parser_i30(CP)

    signals = [
      # signal_name, signal_address
      ("WHL_SPD_FL", "WHL_SPD11"),
      ("WHL_SPD_FR", "WHL_SPD11"),
      ("WHL_SPD_RL", "WHL_SPD11"),
      ("WHL_SPD_RR", "WHL_SPD11"),

      ("YAW_RATE", "ESP12"),

      ("CF_Gway_DrvSeatBeltInd", "CGW4"),

      ("CF_Gway_DrvSeatBeltSw", "CGW1"),
      ("CF_Gway_DrvDrSw", "CGW1"),       # Driver Door
      ("CF_Gway_AstDrSw", "CGW1"),       # Passenger Door
      ("CF_Gway_RLDrSw", "CGW2"),        # Rear left Door
      ("CF_Gway_RRDrSw", "CGW2"),        # Rear right Door
      ("CF_Gway_TurnSigLh", "CGW1"),
      ("CF_Gway_TurnSigRh", "CGW1"),
      ("CF_Gway_ParkBrakeSw", "CGW1"),

      ("CYL_PRES", "ESP12"),

      ("CF_Clu_CruiseSwState", "CLU11"),
      ("CF_Clu_CruiseSwMain", "CLU11"),
      ("CF_Clu_SldMainSW", "CLU11"),
      ("CF_Clu_ParityBit1", "CLU11"),
      ("CF_Clu_VanzDecimal" , "CLU11"),
      ("CF_Clu_Vanz", "CLU11"),
      ("CF_Clu_SPEED_UNIT", "CLU11"),
      ("CF_Clu_DetentOut", "CLU11"),
      ("CF_Clu_RheostatLevel", "CLU11"),
      ("CF_Clu_CluInfo", "CLU11"),
      ("CF_Clu_AmpInfo", "CLU11"),
      ("CF_Clu_AliveCnt1", "CLU11"),

      ("ACCEnable", "TCS13"),
      ("ACC_REQ", "TCS13"),
      ("DriverBraking", "TCS13"),
      ("StandStill", "TCS13"),
      ("PBRAKE_ACT", "TCS13"),

      ("ESC_Off_Step", "TCS15"),
      ("AVH_LAMP", "TCS15"),

      ("CR_Mdps_StrColTq", "MDPS12"),
      ("CF_Mdps_ToiActive", "MDPS12"),
      ("CF_Mdps_ToiUnavail", "MDPS12"),
      ("CF_Mdps_ToiFlt", "MDPS12"),
      ("CR_Mdps_OutTq", "MDPS12"),

      ("SAS_Angle", "SAS11"),
      ("SAS_Speed", "SAS11"),
    ]
    checks = [
      # address, frequency
      ("MDPS12", 50),
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
      ("SAS11", 100),
    ]

    if not CP.openpilotLongitudinalControl:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACCMode", "SCC12"),
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.carFingerprint in FEATURES["use_fca"]:
        signals += [
          ("FCA_CmdAct", "FCA11"),
          ("CF_VSM_Warn", "FCA11"),
        ]
        checks.append(("FCA11", 50))
      else:
        signals += [
          ("AEB_CmdAct", "SCC12"),
          ("CF_VSM_Warn", "SCC12"),
        ]

    if CP.enableBsm:
      signals += [
        ("CF_Lca_IndLeft", "LCA11"),
        ("CF_Lca_IndRight", "LCA11"),
      ]
      checks.append(("LCA11", 50))

    if CP.carFingerprint in (HYBRID_CAR | EV_CAR):
      if CP.carFingerprint in HYBRID_CAR:
        signals.append(("CR_Vcu_AccPedDep_Pos", "E_EMS11"))
      else:
        signals.append(("Accel_Pedal_Pos", "E_EMS11"))
      checks.append(("E_EMS11", 50))
    else:
      signals += [
        ("PV_AV_CAN", "EMS12"),
        ("CF_Ems_AclAct", "EMS16"),
      ]
      checks += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.carFingerprint in FEATURES["use_cluster_gears"]:
      signals.append(("CF_Clu_Gear", "CLU15"))
      checks.append(("CLU15", 5))
    elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
      signals.append(("CUR_GR", "TCU12"))
      checks.append(("TCU12", 100))
    elif CP.carFingerprint in FEATURES["use_elect_gears"]:
      signals.append(("Elect_Gear_Shifter", "ELECT_GEAR"))
      checks.append(("ELECT_GEAR", 20))
    else:
      signals.append(("CF_Lvr_Gear", "LVR12"))
      checks.append(("LVR12", 100))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.carFingerprint in HDA2_CAR:
      signals = [(f"BYTE{i}", "CAM_0x2a4") for i in range(3, 24)]
      checks = [("CAM_0x2a4", 20)]
      return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 6)
    if CP.carFingerprint == CAR.I30:
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


    signals = [
      # signal_name, signal_address
      ("CF_Lkas_LdwsActivemode", "LKAS11"),
      ("CF_Lkas_LdwsSysState", "LKAS11"),
      ("CF_Lkas_SysWarning", "LKAS11"),
      ("CF_Lkas_LdwsLHWarning", "LKAS11"),
      ("CF_Lkas_LdwsRHWarning", "LKAS11"),
      ("CF_Lkas_HbaLamp", "LKAS11"),
      ("CF_Lkas_FcwBasReq", "LKAS11"),
      ("CF_Lkas_HbaSysState", "LKAS11"),
      ("CF_Lkas_FcwOpt", "LKAS11"),
      ("CF_Lkas_HbaOpt", "LKAS11"),
      ("CF_Lkas_FcwSysState", "LKAS11"),
      ("CF_Lkas_FcwCollisionWarning", "LKAS11"),
      ("CF_Lkas_FusionState", "LKAS11"),
      ("CF_Lkas_FcwOpt_USM", "LKAS11"),
      ("CF_Lkas_LdwsOpt_USM", "LKAS11"),
    ]
    checks = [
      ("LKAS11", 100)
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)

  @staticmethod
  def get_can_parser_hda2(CP):
    signals = [
      ("WHEEL_SPEED_1", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_2", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_3", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_4", "WHEEL_SPEEDS"),

      ("ACCELERATOR_PEDAL", "ACCELERATOR"),
      ("GEAR", "ACCELERATOR"),
      ("BRAKE_PRESSED", "BRAKE"),

      ("STEERING_RATE", "STEERING_SENSORS"),
      ("STEERING_ANGLE", "STEERING_SENSORS"),
      ("STEERING_COL_TORQUE", "MDPS"),
      ("STEERING_OUT_TORQUE", "MDPS"),

      ("CRUISE_ACTIVE", "SCC1"),
      ("SET_SPEED", "CRUISE_INFO"),
      ("CRUISE_STANDSTILL", "CRUISE_INFO"),
      ("COUNTER", "CRUISE_BUTTONS"),
      ("CRUISE_BUTTONS", "CRUISE_BUTTONS"),
      ("ADAPTIVE_CRUISE_MAIN_BTN", "CRUISE_BUTTONS"),

      ("DISTANCE_UNIT", "CLUSTER_INFO"),

      ("LEFT_LAMP", "BLINKERS"),
      ("RIGHT_LAMP", "BLINKERS"),

      ("DRIVER_DOOR_OPEN", "DOORS_SEATBELTS"),
      ("DRIVER_SEATBELT_LATCHED", "DOORS_SEATBELTS"),
    ]

    checks = [
      ("WHEEL_SPEEDS", 100),
      ("ACCELERATOR", 100),
      ("BRAKE", 100),
      ("STEERING_SENSORS", 100),
      ("MDPS", 100),
      ("SCC1", 50),
      ("CRUISE_INFO", 50),
      ("CRUISE_BUTTONS", 50),
      ("CLUSTER_INFO", 4),
      ("BLINKERS", 4),
      ("DOORS_SEATBELTS", 4),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 5)

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

    # This check if the signal is found on current bus (last item on the return args), if we get MISSING error in tmux,
    # it tells that the signal has not been seen at all, if we get TIMEOUT error that mean that we are not receiving the
    # message in expected timeframe (signal, expected timeframe), the timeframe is given as Hz, and the errror will trigger
    # if the opendbc/can/can_packer does not receive it in 10 time per expected timeframe eg. 20 = 500 ms
    checks = [
      ("EMS1", 20),
      ("EMS_DCT2", 20),	# True interval 10 ms
      ("VSM2", 20),		# True interval 10 ms
      ("TCS5", 20),		# True interval 20 ms
      ("SAS1", 20),		# True interval 10 ms
      ("EMS2", 20),	  # True interval ? ms
      ("EMS6", 20),	  # True interval ? ms
      ("EMS_DCT1", 20),	# True interval ? ms
      ("ESP2", 20),	  # True interval ? ms
      ("CLU1", 20),		# True interval ? ms
      ("CLU2", 10),		# True interval ? ms
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)
