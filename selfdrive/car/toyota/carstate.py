from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, TSS2_CAR, NO_STOP_TIMER_CAR


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["AGS_1"]['GEAR_SELECTOR']

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.accurate_steer_angle_seen = False
    self.angle_offset = 0.

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["IKE_2"]['DOOR_OPEN_FL'], cp.vl["IKE_2"]['DOOR_OPEN_FR'],
                        cp.vl["IKE_2"]['DOOR_OPEN_RL'], cp.vl["IKE_2"]['DOOR_OPEN_RR']])
    ret.seatbeltUnlatched = cp.vl["IKE_2"]['SEATBELT_DRIVER_UNLATCHED'] != 0

    ret.brakePressed = cp.vl["PCM_CRUISE"]['BRK_ST_OP'] != 0
    ret.brakeLights = bool(cp.vl["DME_2"]['BRAKE_LIGHT_SIGNAL'] or ret.brakePressed)
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = cp.vl["DME_2"]['GAS_PEDAL']
      ret.gasPressed = cp.vl["DME_2"]['GAS_PEDAL'] > 0.05

    ret.wheelSpeeds.fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.01    #Changed this from 0.001 to 0.1 to 0.01 bc longcontrol.py uses this to detect when car is stopped

#    # Some newer models have a more accurate angle measurement in the TORQUE_SENSOR message. Use if non-zero
#    if abs(cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']) > 1e-3:
#      self.accurate_steer_angle_seen = True
#
#    if self.accurate_steer_angle_seen:
#      ret.steeringAngleDeg = cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE'] - self.angle_offset
#      if self.needs_angle_offset:
#        angle_wheel = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']
#        if abs(angle_wheel) > 1e-3:
#          self.needs_angle_offset = False
#          self.angle_offset = ret.steeringAngleDeg - angle_wheel
#    else:
#      ret.steeringAngleDeg = -(cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION'])#
#
#    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']

    if self.CP.carFingerprint == CAR.OLD_CAR: # Steering angle sensor is code differently on BMW
      if cp.vl["SZL_1"]['ANGLE_DIRECTION'] == 0:
        ret.steeringAngleDeg = (cp.vl["SZL_1"]['STEERING_ANGLE'])
      else:
       ret.steeringAngleDeg = -(cp.vl["SZL_1"]['STEERING_ANGLE'])
       #ret.steeringAngle = -(cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION'])
    else:
      ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']

    
    if self.CP.carFingerprint == CAR.OLD_CAR: # Steering rate sensor is code differently on BMW
      if cp.vl["SZL_1"]['VELOCITY_DIRECTION'] == 0:
        ret.steeringRateDeg = (cp.vl["SZL_1"]['STEERING_VELOCITY'])
      else:
        ret.steeringRateDeg = -(cp.vl["SZL_1"]['STEERING_VELOCITY'])
    else:
      ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']

    can_gear = int(cp.vl["AGS_1"]['GEAR_SELECTOR'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["IKE_2"]['BLINKERS'] == 1
    ret.rightBlinker = cp.vl["IKE_2"]['BLINKERS'] == 2

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    #ret.steerWarning = cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]
    #ret.steerWarning = self.CP.carFingerprint not in OLD_CAR and cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]
    ret.steerWarning = False


    if self.CP.carFingerprint == CAR.LEXUS_IS:
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = False
    else:
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]['LOW_SPEED_LOCKOUT'] == 2
    self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_STATE']
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]['CRUISE_ACTIVE'])
    ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]['CRUISE_STATE'] in [1, 2, 3, 4, 5, 6]

    if self.CP.carFingerprint == CAR.PRIUS:
      ret.genericToggle = cp.vl["AUTOPARK_STATUS"]['STATE'] != 0
    else:
      ret.genericToggle = bool(cp.vl["LIGHT_STALK"]['AUTO_HIGH_BEAM'])
    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = cp.vl["DSC_1"]['DSC_OFF'] != 0
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp.vl["EPS_STATUS"]['LKA_STATE']

    #if self.CP.carFingerprint not in NO_EPS_CAR:
    #  self.steer_state = 2
    #else:
    #  self.steer_state = cp.vl["EPS_STATUS"]['LKA_STATE']    
    
    #ret.epsDisabled = True
    ret.epsDisabled = (True if ret.genericToggle == 0 else False)


    if self.CP.carFingerprint in TSS2_CAR:
      ret.leftBlindspot = (cp.vl["BSM"]['L_ADJACENT'] == 1) or (cp.vl["BSM"]['L_APPROACHING'] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]['R_ADJACENT'] == 1) or (cp.vl["BSM"]['R_APPROACHING'] == 1)

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
      ("DOOR_OPEN_FL", "IKE_2", 1),     #Imported from BMW
      ("DOOR_OPEN_FR", "IKE_2", 1),     #Imported from BMW
      ("DOOR_OPEN_RL", "IKE_2", 1),     #Imported from BMW
      ("DOOR_OPEN_RR", "IKE_2", 1),     #Imported from BMW
      ("SEATBELT_DRIVER_UNLATCHED", "IKE_2", 1),      #Imported from BMW
      ("DSC_OFF", "DSC_1", 1),      #Imported from BMW
      ("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),      #Unneccasary?
      ("STEERING_VELOCITY", "SZL_1", 0),      #Imported from BMW
      ("ANGLE_DIRECTION", "SZL_1", 0),      #Imported from BMW
      ("VELOCITY_DIRECTION", "SZL_1", 0),     #Imported from BMW
      ("CRUISE_ACTIVE", "PCM_CRUISE", 0),
      ("CRUISE_STATE", "PCM_CRUISE", 0),
      ("BRK_ST_OP", "PCM_CRUISE", 0),
      ("GAS_RELEASED", "PCM_CRUISE", 1),      #Check this OUT is it neccessary anymore because made it different above code!!!
      ("RESUME_BTN", "DME_2", 0),     #Imported from BMW
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("BLINKERS", "IKE_2", 0),   # 0 is no blinkers, Imported from BMW
      ("LKA_STATE", "EPS_STATUS", 0),
      ("BRAKE_LIGHT_SIGNAL", "DME_2", 0),      #Imported from BMW
      ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
      ("ACCEL_CMD", "ACC_CONTROL", 0),
    ]

    checks = [
        ("DSC_1", 40),
        ("DME_2", 33),
        ("WHEEL_SPEEDS", 80),
        ("IKE_2", 33)
    ]

    if CP.carFingerprint == CAR.LEXUS_IS:
      signals.append(("MAIN_ON", "DSU_CRUISE", 0))
      signals.append(("SET_SPEED", "DSU_CRUISE", 0))
      checks.append(("DSU_CRUISE", 5))
    else:
      signals.append(("MAIN_ON", "PCM_CRUISE_2", 0))
      signals.append(("SET_SPEED", "PCM_CRUISE_2", 0))
      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0))
      checks.append(("PCM_CRUISE_2", 33))

    if CP.carFingerprint == CAR.PRIUS:
      signals += [("STATE", "AUTOPARK_STATUS", 0)]

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR", 0))
      checks.append(("GAS_SENSOR", 50))

    if CP.carFingerprint in TSS2_CAR:
      signals += [("L_ADJACENT", "BSM", 0)]
      signals += [("L_APPROACHING", "BSM", 0)]
      signals += [("R_ADJACENT", "BSM", 0)]
      signals += [("R_APPROACHING", "BSM", 0)]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      ("FORCE", "PRE_COLLISION", 0),
      ("PRECOLLISION_ACTIVE", "PRE_COLLISION", 0)
    ]

    # use steering message to check if panda is connected to frc
    checks = [
      ("STEERING_LKA", 42)
    ]
    
    #if CP.carFingerprint not in NO_EPS_CAR:
    #  checks = [("STEERING_LKA", 42)]
    #else:
    #  checks = []
    
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
