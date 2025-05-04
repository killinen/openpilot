# import copy
from cereal import car
from selfdrive.car.hyundai.values import DBC #, STEER_THRESHOLD, FEATURES, EV_CAR, HYBRID_CAR
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
# from opendbc.can.can_define import CANDefine
# from selfdrive.config import Conversions as CV

GearShifter = car.CarState.GearShifter

class CarState(CarStateBase):
  # pylint: disable=useless-super-delegation
  def __init__(self, CP):
    super().__init__(CP)
    # can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    #if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
    #  self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    #elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
    #  self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    #else:  # preferred and elect gear methods use same definition
    #  self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    # SSC assembly integrity tracking
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.ssc_steer_angle = True    # This is first set to false so we ignore the SCC angle stuff
    self.angle_offset = 0.0
    # Initialize variables to store the min and max error values
    self.steeringAngle_aligned = False
    self.min_error = 0.0
    self.max_error = 0.0

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["CLU2"]['CF_Clu_DrvDrSw'], cp.vl["CLU2"]['CF_Clu_AstDrSw']])

    ret.seatbeltUnlatched = cp.vl["CLU2"]['CF_Clu_DrvSeatBeltSw'] == 1

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["TCS5"]["WHEEL_FL"],
      cp.vl["TCS5"]["WHEEL_FL"],
      cp.vl["TCS5"]["WHEEL_FL"],
      cp.vl["TCS5"]["WHEEL_FL"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringAngleDeg = cp.vl["SAS1"]['SAS_Angle']
    ret.steeringRateDeg = cp.vl["SAS1"]['SAS_Speed']
    ret.yawRate = cp.vl["ESP2"]['YAW_RATE']
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["CLU2"]['CF_Clu_TurnSigLh'],
                                                            cp.vl["CLU2"]['CF_Clu_TurnSigRh'])

    ret.gasPressed = cp.vl["EMS6"]['CF_Ems_AclAct'] > 0.05

    ret.steeringTorque = cp.vl["VSM2"]["CR_Mdps_StrTq"]
    ret.steeringTorqueOut = cp.vl["VSM2"]["CR_Mdps_OutTq"]
    ret.steeringTorqueEps = cp_cam.vl["STEERING_STATUS"]['STEERING_TORQUE']

    # emulate driver steering torque - allows lane change assist on blinker hold
    ret.steeringPressed = ret.gasPressed    # i30 with SSC doesn't have separate torque sensor, so lightly pressing the gas indicates driver intention to change lane
    #if ret.steeringPressed and ret.leftBlinker:
    #  ret.steeringTorque = 1
    #elif ret.steeringPressed and  ret.rightBlinker:
    #  ret.steeringTorque = -1
    #else:
    #  ret.steeringTorque = 0

    # ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerWarning = False


    # EPS and SSC angle difference tracking for assembly integrity monitoring
    if self.ssc_steer_angle:
      if self.needs_angle_offset:
        self.angle_offset = (cp_cam.vl["STEERING_STATUS"]['STEERING_ANGLE']) - ret.steeringAngleDeg
        self.needs_angle_offset = False
      else:
        # After angle_offset has been set, start measuring aligned SSC angle
        self.steeringAngleDegSSC = (cp_cam.vl["STEERING_STATUS"]['STEERING_ANGLE']) - self.angle_offset
        if abs(ret.steeringAngleDeg - self.steeringAngleDegSSC) < 0.1:
          self.steeringAngle_aligned = True
        # Calculate the error (difference) between the two sensor readings
        ret.steeringAngleDegError = self.steeringAngleDegSSC - ret.steeringAngleDeg

        # Track the minimum and maximum error values
        if self.steeringAngle_aligned:
          self.max_error = max(self.max_error, ret.steeringAngleDegError)
          self.min_error = min(self.min_error, ret.steeringAngleDegError)

        ret.steeringAngleOffsetDeg = self.max_error - self.min_error

    # cruise state
    # if self.CP.openpilotLongitudinalControl:
    #   ret.cruiseState.available = cp.vl["TCS13"]['ACCEnable'] == 0
    #   ret.cruiseState.enabled = cp.vl["TCS13"]['ACC_REQ'] == 1
    #   ret.cruiseState.standstill = cp.vl["TCS13"]['StandStill'] == 1
    # else:
    #   ret.cruiseState.available = cp.vl["SCC11"]['MainMode_ACC'] == 1
    #   ret.cruiseState.enabled = cp.vl["SCC12"]['ACCMode'] != 0
    #   ret.cruiseState.standstill = cp.vl["SCC11"]['SCCInfoDisplay'] == 4.
    # if ret.cruiseState.enabled:
    #   speed_conv = CV.MPH_TO_MS if cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] else CV.KPH_TO_MS
    #   ret.cruiseState.speed = cp.vl["SCC11"]['VSetDis'] * speed_conv
    # else:
    #   ret.cruiseState.speed = 0
    ret.cruiseState.available = cp.vl["EMS6"]['CRUISE_LAMP_M'] != 0
    ret.cruiseState.enabled = bool(cp.vl["EMS6"]['CRUISE_LAMP_S'])
    ret.cruiseState.standstill = False
    ret.cruiseState.speed = 0

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["EMS_DCT2"]['BRAKE_ACT'] == 2
    # ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2 # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY

    # if self.CP.carFingerprint in (HYBRID_CAR | EV_CAR):
    #   if self.CP.carFingerprint in HYBRID_CAR:
    #     ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
    #   else:
    #     ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
    #   ret.gasPressed = ret.gas > 0
    # else:
    #   ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
    #   ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    ret.gas = cp.vl["EMS_DCT1"]['PV_AV_CAN']
    # ret.gasPressed = cp.vl["EMS6"]['CF_Ems_AclAct'] > 0.05

    # # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # # as this seems to be standard over all cars, but is not the preferred method.
    # if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
    #   gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    # elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
    #   gear = cp.vl["TCU12"]["CUR_GR"]
    # elif self.CP.carFingerprint in FEATURES["use_elect_gears"]:
    #   gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    # else:
    #   gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    ret.gearShifter = GearShifter.reverse if cp.vl["CLU2"]['CF_Clu_SwiGearR'] else GearShifter.drive	# Force D-gear otherwise because my car is manual

    # if not self.CP.openpilotLongitudinalControl:
    #   if self.CP.carFingerprint in FEATURES["use_fca"]:
    #     ret.stockAeb = cp.vl["FCA11"]["FCA_CmdAct"] != 0
    #     ret.stockFcw = cp.vl["FCA11"]["CF_VSM_Warn"] == 2
    #   else:
    #     ret.stockAeb = cp.vl["SCC12"]["AEB_CmdAct"] != 0
    #     ret.stockFcw = cp.vl["SCC12"]["CF_VSM_Warn"] == 2

    # if self.CP.enableBsm:
    #   ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
    #   ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    # save the entire LKAS11 and CLU11
    # self.lkas11 = copy.copy(cp_cam.vl["LKAS11"])
    # self.clu11 = copy.copy(cp.vl["CLU11"])
    # self.park_brake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    # self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    # self.brake_error = cp.vl["TCS13"]["ACCEnable"] != 0 # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED

    # TODO: You should make something up for these when have the time
    self.brake_error = False
    self.park_brake = False

#    self.clu11 = {
#      "CF_Clu_CruiseSwState": 3,  # Example state of cruise switch (range 0-7)
#      "CF_Clu_CruiseSwMain": 1,   # Cruise switch main is active (0 or 1)
#      "CF_Clu_SldMainSW": 0,      # SLD main switch inactive (0 or 1)
#      "CF_Clu_ParityBit1": 1,     # Parity bit set (0 or 1)
#      "CF_Clu_VanzDecimal": 0.125, # Vehicle speed decimal part (range 0.0-0.375)
#      "CF_Clu_Vanz": 100,         # Vehicle speed (km/h, range 0-255.5)
#      "CF_Clu_SPEED_UNIT": 0,     # Speed unit is km/h (0 for km/h, 1 for MPH)
#      "CF_Clu_DetentOut": 1,      # Detent output active (0 or 1)
#      "CF_Clu_RheostatLevel": 15, # Rheostat level (range 0-31)
#      "CF_Clu_CluInfo": 0,        # CluInfo is inactive (0 or 1)
#      "CF_Clu_AmpInfo": 0,        # AmpInfo is inactive (0 or 1)
#      "CF_Clu_AliveCnt1": 5       # Alive counter (range 0-15)
#    }
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = cp.vl["CLU1"]["CF_Clu_CruiseSwState"]

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("WHEEL_FL", "TCS5", 0),                #Imported from i30
      ("WHEEL_FR", "TCS5", 0),                #Imported from i30
      ("WHEEL_RL", "TCS5", 0),                #Imported from i30
      ("WHEEL_RR", "TCS5", 0),                #Imported from i30

      ("YAW_RATE", "ESP2", 0),                #Imported from i30

      ("CF_Clu_DrvSeatBeltSw", "CLU2", 0),    #Imported from i30
      ("CF_Clu_DrvDrSw", "CLU2", 1),          #Imported from i30       # Driver Door
      ("CF_Clu_AstDrSw", "CLU2", 1),          #Imported from i30,      # Passenger door
#      ("CF_Gway_RLDrSw", "CGW2", 0),         # Rear reft door
#      ("CF_Gway_RRDrSw", "CGW2", 0),         # Rear right door
      ("CF_Clu_TurnSigLh", "CLU2", 0),        #Imported from i30
      ("CF_Clu_TurnSigRh", "CLU2", 0),        #Imported from i30
      ("CF_Clu_SwiGearR", "CLU2", 0),         #Imported from i30
      #("CF_Gway_ParkBrakeSw", "CGW1", 0),

      ("CF_Clu_CruiseSwState", "CLU1", 0),    #Imported from i30
      #("CF_Clu_CruiseSwMain", "CLU1", 0),
      #("CF_Clu_SldMainSW", "CLU1", 0),
      #("CF_Clu_ParityBit1", "CLU1", 0),
      #("CF_Clu_VanzDecimal" , "CLU1", 0),
      #("CF_Clu_Vanz", "CLU1", 0),
      #("CF_Clu_SPEED_UNIT", "CLU1", 0),
      #("CF_Clu_DetentOut", "CLU3", 0),
      #("CF_Clu_RheostatLevel", "CLU3", 0),
      #("CF_Clu_CluInfo", "CLU3", 0),
      #("CF_Clu_AmpInfo", "CLU11", 0),
      #("CF_Clu_AliveCnt1", "CLU11", 0),

      ("CRUISE_LAMP_M", "EMS6", 0),           #Imported from i30
      ("CRUISE_LAMP_S", "EMS6", 0),           #Imported from i30
      # ("ACCEnable", "TCS13", 0),
      # ("ACC_REQ", "TCS13", 0),
      # ("BrakeLight", "TCS13", 0),
      # ("DriverBraking", "TCS13", 0),
      # ("StandStill", "TCS13", 0),
      # ("PBRAKE_ACT", "TCS13", 0),
      ("BRAKE_ACT", "EMS2", 0),
      ("BRAKE_ACT", "EMS_DCT2", 0),

      # ("ESC_Off_Step", "TCS15", 0),
      # ("AVH_LAMP", "TCS15", 0),

      # ("CF_Lvr_GearInf", "LVR11", 0),        # Transmission Gear (0 = N or P, 1-8 = Fwd, 14 = Rev)

      # ("CR_Mdps_StrColTq", "MDPS12", 0),
      # ("CF_Mdps_ToiActive", "MDPS12", 0),
      # ("CF_Mdps_ToiUnavail", "MDPS12", 0),
      # ("CF_Mdps_FailStat", "MDPS12", 0),
      # ("CR_Mdps_OutTq", "MDPS12", 0),

      ("PV_AV_CAN", "EMS_DCT1", 0),           #Imported from i30
      ("CF_Ems_AclAct", "EMS6", 1),           #Imported from i30
      ("CR_Mdps_StrTq", "VSM2", 0),           #Imported from i30
      ("CR_Mdps_OutTq", "VSM2", 0),           #Imported from i30

      ("SAS_Angle", "SAS1", 0),               #Imported from i30
      ("SAS_Speed", "SAS1", 0),               #Imported from i30

      # ("MainMode_ACC", "SCC11", 0),
      # ("VSetDis", "SCC11", 0),
      # ("SCCInfoDisplay", "SCC11", 0),
      # ("ACC_ObjDist", "SCC11", 0),
      # ("ACCMode", "SCC12", 1),
    ]

    checks = [			# TODO: change the interval to correct when want to do some polishing
        ("EMS_DCT2", 20),	# True interval 10 ms
        ("VSM2", 20),		# True interval 10 ms
        ("TCS5", 20),		# True interval 20 ms
        ("SAS1", 20),		# True interval 10 ms
        ("EMS2", 20),	  # True interval ? ms
        ("EMS6", 20),	  # True interval ? ms
        ("EMS_DCT1", 20),	# True interval ? ms
        ("ESP2", 20),	  # True interval ? ms
        ("CLU1", 20),		# True interval ? ms
        ("CLU2", 20),		# True interval ? ms
        ("CLU3", 20)		# True interval ? ms
    ]

    # if not CP.openpilotLongitudinalControl:
    #   signals += [
    #     ("MainMode_ACC", "SCC11", 0),
    #     ("VSetDis", "SCC11", 0),
    #     ("SCCInfoDisplay", "SCC11", 0),
    #     ("ACC_ObjDist", "SCC11", 0),
    #     ("ACCMode", "SCC12", 1),
    #   ]

    #   checks += [
    #     ("SCC11", 50),
    #     ("SCC12", 50),
    #   ]

    #   if CP.carFingerprint in FEATURES["use_fca"]:
    #     signals += [
    #       ("FCA_CmdAct", "FCA11", 0),
    #       ("CF_VSM_Warn", "FCA11", 0),
    #     ]
    #     checks += [("FCA11", 50)]
    #   else:
    #     signals += [
    #       ("AEB_CmdAct", "SCC12", 0),
    #       ("CF_VSM_Warn", "SCC12", 0),
    #     ]

    # if CP.enableBsm:
    #   signals += [
    #     ("CF_Lca_IndLeft", "LCA11", 0),
    #     ("CF_Lca_IndRight", "LCA11", 0),
    #   ]
    #   checks += [("LCA11", 50)]

    # if CP.carFingerprint in (HYBRID_CAR | EV_CAR):
    #   if CP.carFingerprint in HYBRID_CAR:
    #     signals += [
    #       ("CR_Vcu_AccPedDep_Pos", "E_EMS11", 0)
    #     ]
    #   else:
    #     signals += [
    #       ("Accel_Pedal_Pos", "E_EMS11", 0)
    #     ]
    #   checks += [
    #     ("E_EMS11", 50),
    #   ]
    # else:
    #   signals += [
    #     ("PV_AV_CAN", "EMS12", 0),
    #     ("CF_Ems_AclAct", "EMS16", 0),
    #   ]
    #   checks += [
    #     ("EMS12", 100),
    #     ("EMS16", 100),
    #   ]

    # if CP.carFingerprint in FEATURES["use_cluster_gears"]:
    #   signals += [
    #     ("CF_Clu_Gear", "CLU15", 0),
    #   ]
    #   checks += [
    #     ("CLU15", 5)
    #   ]
    # elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
    #   signals += [
    #     ("CUR_GR", "TCU12", 0)
    #   ]
    #   checks += [
    #     ("TCU12", 100)
    #   ]
    # elif CP.carFingerprint in FEATURES["use_elect_gears"]:
    #   signals += [("Elect_Gear_Shifter", "ELECT_GEAR", 0)]
    #   checks += [("ELECT_GEAR", 20)]
    # else:
    #   signals += [
    #     ("CF_Lvr_Gear", "LVR12", 0)
    #   ]
    #   checks += [
    #     ("LVR12", 100)
    #   ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("STEERING_TORQUE", "STEERING_STATUS", 0),
      ("STEERING_ANGLE", "STEERING_STATUS", 0),
      # ("CF_Lkas_LdwsActivemode", "LKAS11", 0),
      # ("CF_Lkas_LdwsSysState", "LKAS11", 0),
      # ("CF_Lkas_SysWarning", "LKAS11", 0),
      # ("CF_Lkas_LdwsLHWarning", "LKAS11", 0),
      # ("CF_Lkas_LdwsRHWarning", "LKAS11", 0),
      # ("CF_Lkas_HbaLamp", "LKAS11", 0),
      # ("CF_Lkas_FcwBasReq", "LKAS11", 0),
      # ("CF_Lkas_HbaSysState", "LKAS11", 0),
      # ("CF_Lkas_FcwOpt", "LKAS11", 0),
      # ("CF_Lkas_HbaOpt", "LKAS11", 0),
      # ("CF_Lkas_FcwSysState", "LKAS11", 0),
      # ("CF_Lkas_FcwCollisionWarning", "LKAS11", 0),
      # ("CF_Lkas_FusionState", "LKAS11", 0),
      # ("CF_Lkas_FcwOpt_USM", "LKAS11", 0),
      # ("CF_Lkas_LdwsOpt_USM", "LKAS11", 0),
    ]

    checks = [
      ("STEERING_STATUS", 20)    # Checks if SSC is connected
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 1)
