import cereal.messaging as messaging
from cereal import car
from common.numpy_fast import mean
from common.filter_simple import FirstOrderFilter
from common.op_params import opParams
from common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, TSS2_CAR


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["AGS_1"]["GEAR_SELECTOR"]

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)
    self.has_zss = CP.hasZss

    self.low_speed_lockout = False
    self.acc_type = 1

    # Toyota Distance Button
    op_params = opParams()
    self.enable_distance_btn = op_params.get('toyota_distance_btn')
    self.distance_btn = 0
    self.distance_lines = 0
    if self.enable_distance_btn:
      # Previously was publishing from UI
      self.pm = messaging.PubMaster(['dynamicFollowButton'])

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = False		# E39 does not have door info in CAN
    ret.seatbeltUnlatched = False		# E39 does not have seatbelt info in CAN

    ret.brakePressed = cp.vl["PCM_CRUISE"]["BRK_ST_OP"] != 0
    ret.brakeHoldActive = False   # E39 does not have brakeHoldActive, maybe this variable could be levereged in SnG
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = cp.vl["DME_2"]["GAS_PEDAL"]
      ret.gasPressed = cp.vl["DME_2"]["GAS_PEDAL"] > 0.05

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.01    #Changed this from 0.001 to 0.1 to 0.01 bc longcontrol.py uses this to detect when car is stopped

    # ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    # torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # zss_angle_deg = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"] if self.has_zss else 0.

    if self.CP.carFingerprint == CAR.BMW_E39: # Steering angle sensor is code differently on BMW
      if cp.vl["SZL_1"]["ANGLE_DIRECTION"] == 0:
        ret.steeringAngleDeg = (cp.vl["SZL_1"]["STEERING_ANGLE"])
      else:
       ret.steeringAngleDeg = -(cp.vl["SZL_1"]["STEERING_ANGLE"])
    else:
      ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]

    # # Some newer models have a more accurate angle measurement in the TORQUE_SENSOR message. Use if non-zero or ZSS
    # # Also only get offset when ZSS comes up in case it's slow to start sending messages
    # if abs(torque_sensor_angle_deg) > 1e-3 or (self.has_zss and abs(zss_angle_deg) > 1e-3):
    #   self.accurate_steer_angle_seen = True

    # if self.accurate_steer_angle_seen:
    #   acc_angle_deg = zss_angle_deg if self.has_zss else torque_sensor_angle_deg
    #   # Offset seems to be invalid for large steering angles
    #   if abs(ret.steeringAngleDeg) < 90:
    #     self.angle_offset.update(acc_angle_deg - ret.steeringAngleDeg)

    #   if self.angle_offset.initialized:
    #     ret.steeringAngleOffsetDeg = self.angle_offset.x
    #     ret.steeringAngleDeg = acc_angle_deg - self.angle_offset.x

    # ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    if self.CP.carFingerprint == CAR.BMW_E39: # Steering rate sensor is code differently on BMW
      if cp.vl["SZL_1"]["VELOCITY_DIRECTION"] == 0:
        ret.steeringRateDeg = (cp.vl["SZL_1"]["STEERING_VELOCITY"])
      else:
        ret.steeringRateDeg = -(cp.vl["SZL_1"]["STEERING_VELOCITY"])
    else:
      ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]

    can_gear = int(cp.vl["AGS_1"]["GEAR_SELECTOR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["IKE_2"]["BLINKERS"] == 1
    ret.rightBlinker = cp.vl["IKE_2"]["BLINKERS"] == 2

    # Emulate driver steering torque - allows lane change assist on blinker hold
    ret.steeringPressed = ret.gasPressed # E-series doesn't have torque sensor, so lightly pressing the gas indicates driver intention
    if ret.steeringPressed and ret.leftBlinker:
      ret.steeringTorque = 1
    elif ret.steeringPressed and  ret.rightBlinker:
      ret.steeringTorque = -1
    else:
      ret.steeringTorque = 0

    # ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    # ret.steeringTorqueEps = cp.vl["STEERING_STATUS"]['STEERING_TORQUE']     # StepperServoCAN produced torque

    if self.CP.carFingerprint in [CAR.LEXUS_IS, CAR.LEXUS_RC]:
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
    else:
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS

    if self.CP.carFingerprint in TSS2_CAR:
      self.acc_type = cp_cam.vl["ACC_CONTROL"]["ACC_TYPE"]

    if self.enable_distance_btn:
      if self.CP.carFingerprint in TSS2_CAR:
        self.distance_btn = 1 if cp_cam.vl["ACC_CONTROL"]["DISTANCE"] == 1 else 0
      elif self.CP.smartDsu:
        self.distance_btn = 1 if cp.vl["SDSU"]["FD_BUTTON"] == 1 else 0

      distance_lines = cp.vl["PCM_CRUISE_SM"]["DISTANCE_LINES"] - 1
      if distance_lines in range(3) and distance_lines != self.distance_lines:
        dat = messaging.new_message('dynamicFollowButton')
        dat.dynamicFollowButton.status = distance_lines
        self.pm.send('dynamicFollowButton', dat)
        self.distance_lines = distance_lines

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in [CAR.LEXUS_IS, CAR.LEXUS_RC]) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]["CRUISE_STATE"] in [1, 2, 3, 4, 5, 6]

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = cp.vl["DSC_1"]['DSC_OFF'] != 0
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]["LKA_STATE"]

    # Add variable for ACC-mode only
    ret.epsDisabled = (True if ret.genericToggle == 0 else False)

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    return ret

  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("STEERING_ANGLE", "SZL_1", 0),     #Imported from BMW
      ("GEAR_SELECTOR", "AGS_1", 0),      #Imported from BMW
      ("GEAR", "AGS_1", 0),      #Imported from BMW
      ("BRAKE_LIGHT_SIGNAL", "DSC_1", 0),     #Imported from BMW
      ("GAS_PEDAL", "DME_2", 0),      #Imported from BMW
      ("CRUISE_I_O", "DME_2", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),      #Imported from BMW
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),      #Imported from BMW
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),      #Imported from BMW
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),      #Imported from BMW
      ("DSC_OFF", "DSC_1", 1),      #Imported from BMW
      #("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),      #Unneccasary?
      ("STEERING_VELOCITY", "SZL_1", 0),      #Imported from BMW
      ("ANGLE_DIRECTION", "SZL_1", 0),      #Imported from BMW
      ("VELOCITY_DIRECTION", "SZL_1", 0),     #Imported from BMW
      ("CRUISE_ACTIVE", "PCM_CRUISE", 0),
      ("CRUISE_STATE", "PCM_CRUISE", 0),
      ("BRK_ST_OP", "PCM_CRUISE", 0),
      #("GAS_RELEASED", "PCM_CRUISE", 0),      #Check this OUT is it neccessary anymore because made it different above code!!!
      ("RESUME_BTN", "DME_2", 0),     #Imported from BMW
      #("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      #("STEERING_TORQUE", "STEERING_STATUS", 0),
      #("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("BLINKERS", "IKE_2", 0),   # 0 is no blinkers, Imported from BMW
      ("LKA_STATE", "EPS_STATUS", 0),
      ("BRAKE_LIGHT_SIGNAL", "DME_2", 0),      #Imported from BMW
      ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
      ("DISTANCE_LINES", "PCM_CRUISE_SM", 0),
    ]

    checks = [
        ("DSC_1", 40),
        ("DME_2", 33),
        ("WHEEL_SPEEDS", 80),
        ("IKE_2", 33),
        ("SZL_1", 40),
        ("AGS_1", 40),
        ("PCM_CRUISE", 40),
        ("LIGHT_STALK", 40),
        ("EPS_STATUS", 40),
        ("PCM_CRUISE_SM", 40),
        #("STEERING_STATUS", 40),
    ]

    if CP.carFingerprint in [CAR.LEXUS_IS, CAR.LEXUS_RC]:
      signals.append(("MAIN_ON", "DSU_CRUISE", 0))
      signals.append(("SET_SPEED", "DSU_CRUISE", 0))
      checks.append(("DSU_CRUISE", 5))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2", 0))
      signals.append(("SET_SPEED", "PCM_CRUISE_2", 0))
      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0))
      checks.append(("PCM_CRUISE_2", 33))

    if CP.hasZss:
      signals.append(("ZORRO_STEER", "SECONDARY_STEER_ANGLE", 0))
      checks.append(("SECONDARY_STEER_ANGLE", 80))

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR", 0))
      checks.append(("GAS_SENSOR", 50))

    if CP.enableBsm:
      signals += [
        ("L_ADJACENT", "BSM", 0),
        ("L_APPROACHING", "BSM", 0),
        ("R_ADJACENT", "BSM", 0),
        ("R_APPROACHING", "BSM", 0),
      ]
      checks += [
        ("BSM", 1)
      ]

    if CP.smartDsu:
      signals.append(("FD_BUTTON", "SDSU", 0))
      checks.append(("SDSU", 33))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      ("FORCE", "PRE_COLLISION", 0),
      ("PRECOLLISION_ACTIVE", "PRE_COLLISION", 0),
    ]

    # use steering message to check if panda is connected to frc
    checks = [
      ("STEERING_LKA", 42),
      ("PRE_COLLISION", 0), # TODO: figure out why freq is inconsistent
    ]

    if CP.carFingerprint in TSS2_CAR:
      signals.append(("ACC_TYPE", "ACC_CONTROL", 0))
      signals.append(("DISTANCE", "ACC_CONTROL", 0))
      checks.append(("ACC_CONTROL", 33))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
