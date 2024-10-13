from cereal import car
from common.numpy_fast import mean
#from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.toyota.values import CAR, DBC, STEER_THRESHOLD, TSS2_CAR, NO_STOP_TIMER_CAR

GearShifter = car.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    #can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    #self.shifter_values = can_define.dv["AGS_1"]['GEAR_SELECTOR']

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.needs_angle_offset = True
    self.accurate_steer_angle_seen = False    # This is first set to false so we ignore the SCC angle stuff
    self.angle_offset = 0.
    # Initialize variables to store the min and max error values
    self.steeringAngle_aligned = False
    self.min_error = 0
    self.max_error = 0


  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.doorOpen = any([cp.vl["CLU2"]['CF_Clu_DrvDrSw'], cp.vl["CLU2"]['CF_Clu_AstDrSw']])
    ret.seatbeltUnlatched = cp.vl["CLU2"]['CF_Clu_DrvSeatBeltSw'] == 1

    ret.brakePressed = cp.vl["EMS_DCT2"]['BRAKE_ACT'] == 2
    ret.brakeLights = bool(cp.vl["EMS2"]['BRAKE_ACT'] == 2 or ret.brakePressed)
    if self.CP.enableGasInterceptor:
      ret.gas = (cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS'] + cp.vl["GAS_SENSOR"]['INTERCEPTOR_GAS2']) / 2.
      ret.gasPressed = ret.gas > 15
    else:
      ret.gas = cp.vl["EMS_DCT1"]['PV_AV_CAN']
      ret.gasPressed = cp.vl["EMS6"]['CF_Ems_AclAct'] > 0.05

    ret.wheelSpeeds.fl = cp.vl["TCS5"]['WHEEL_FL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = cp.vl["TCS5"]['WHEEL_FR'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = cp.vl["TCS5"]['WHEEL_RL'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = cp.vl["TCS5"]['WHEEL_RR'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = ret.vEgoRaw < 0.01    #Changed this from 0.001 to 0.1 to 0.01 bc longcontrol.py uses this to detect when car is stopped

    # Some newer models have a more accurate angle measurement in the TORQUE_SENSOR message. Use if non-zero
#    if abs(cp.vl["STEER_TORQUE_SENSOR"]['STEER_ANGLE']) > 1e-3:
#      self.accurate_steer_angle_seen = True
#
#    if self.accurate_steer_angle_seen:
#      if self.CP.hasZss:
#        ret.steeringAngleDeg = cp.vl["SECONDARY_STEER_ANGLE"]['ZORRO_STEER'] - self.angle_offset
#      else:
#        ret.steeringAngleDegSSC = cp.vl["STEERING_STATUS"]['STEERING_ANGLE'] - self.angle_offset
#      if self.needs_angle_offset:
##        angle_wheel = cp.vl["SZL_1"]['STEERING_ANGLE']
#        if cp.vl["SZL_1"]['ANGLE_DIRECTION'] == 0:
#          angle_wheel = (cp.vl["SZL_1"]['STEERING_ANGLE'])
#        else:
#          angle_wheel = -(cp.vl["SZL_1"]['STEERING_ANGLE'])
#        if abs(angle_wheel) > 1e-3:
#          self.needs_angle_offset = False
#          self.angle_offset = ret.steeringAngleDegSSC - angle_wheel

    if self.accurate_steer_angle_seen:
      if self.CP.hasZss:
        ret.steeringAngleDeg = cp.vl["SECONDARY_STEER_ANGLE"]['ZORRO_STEER'] - self.angle_offset
      # else:
      #   ret.steeringAngleDegSSC = cp.vl["STEERING_STATUS"]['STEERING_ANGLE'] - self.angle_offset
      if self.needs_angle_offset:
        if cp.vl["SZL_1"]['ANGLE_DIRECTION'] == 0:
          angle_wheel = (cp.vl["SZL_1"]['STEERING_ANGLE'])
        else:
          angle_wheel = -(cp.vl["SZL_1"]['STEERING_ANGLE'])
        if abs(angle_wheel) > 1e-3:
          self.needs_angle_offset = False
          ret.steeringAngleDeg = angle_wheel
          self.angle_offset = cp.vl["STEERING_STATUS"]['STEERING_ANGLE'] - angle_wheel
      else:
        # After angle_offset has been set, start measuring aligned SSC angle
        ret.steeringAngleDegSSC = cp.vl["STEERING_STATUS"]['STEERING_ANGLE'] - self.angle_offset
        if abs(ret.steeringAngleDeg - ret.steeringAngleDegSSC) < 0.1:
          self.steeringAngle_aligned = True
        ## Calculate the error (difference) between the two sensor readings
        #ret.steeringAngleDegError = (ret.steeringAngleDegSSC * 0.96) -  ret.steeringAngleDeg  

        ## Track the minimum and maximum error values
        #if abs(ret.steeringAngleDeg) < 90:
        #  self.max_error = max(self.max_error, ret.steeringAngleDegError)
        #  self.min_error = min(self.min_error, ret.steeringAngleDegError)

        #ret.steeringAngleDegDivergence = self.max_error - self.min_error


    if self.CP.carFingerprint == CAR.OLD_CAR:	# Different logik for OLD_CAR
       ret.steeringAngleDeg = -(cp.vl["SAS1"]['SAS_Angle'])		# Negate factor to make the code align with original BMW steerlogik
    else:
      ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_ANGLE'] + cp.vl["STEER_ANGLE_SENSOR"]['STEER_FRACTION']

    if self.CP.carFingerprint == CAR.OLD_CAR: # Steering rate sensor is code differently on i30
        ret.steeringRateDeg = -(cp.vl["SAS1"]['SAS_Speed'])
    else:
      ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]['STEER_RATE']

    if self.steeringAngle_aligned:
      # Calculate the error (difference) between the two sensor readings
      ret.steeringAngleDegError = (ret.steeringAngleDegSSC * 0.96) -  ret.steeringAngleDeg

      # Track the minimum and maximum error values
      if abs(ret.steeringAngleDeg) < 90:
        self.max_error = max(self.max_error, ret.steeringAngleDegError)
        self.min_error = min(self.min_error, ret.steeringAngleDegError)

      ret.steeringAngleDegDivergence = self.max_error - self.min_error

    #can_gear = int(cp.vl["AGS_1"]['GEAR_SELECTOR'])
    #ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.gearShifter = GearShifter.drive		# Force D-gear because my car is manual
    ret.leftBlinker, ret.rightBlinker = self.update_blinker(50, cp.vl["CLU2"]['CF_Clu_TurnSigLh'],
                                                            cp.vl["CLU2"]['CF_Clu_TurnSigRh'])

    # ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']

    # emulate driver steering torque - allows lane change assist on blinker hold
    ret.steeringPressed = ret.gasPressed # E-series doesn't have torque sensor, so lightly pressing the gas indicates driver intention
    if ret.steeringPressed and ret.leftBlinker:
      ret.steeringTorque = 1
    elif ret.steeringPressed and  ret.rightBlinker:
      ret.steeringTorque = -1
    else:
      ret.steeringTorque = 0

    ret.steeringTorqueEps = cp.vl["VSM2"]['CR_Mdps_OutTq']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    # ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    #ret.steerWarning = cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]
    #ret.steerWarning = self.CP.carFingerprint not in OLD_CAR and cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]
    ret.steerWarning = False

    if self.CP.carFingerprint == CAR.LEXUS_IS:
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]['MAIN_ON'] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]['SET_SPEED'] * CV.KPH_TO_MS
      self.low_speed_lockout = False
    else:
      ret.cruiseState.available = cp.vl["EMS6"]['CRUISE_LAMP_M'] != 0
      ret.cruiseState.speed = 0 * CV.KPH_TO_MS    # I don't have (yet) cruise speed
      self.low_speed_lockout = False
    self.pcm_acc_status = 0     # Not sure i if this is correct for i30
    if self.CP.carFingerprint in NO_STOP_TIMER_CAR or self.CP.enableGasInterceptor:
      # ignore standstill in hybrid vehicles, since pcm allows to restart without
      # receiving any special command. Also if interceptor is detected
      ret.cruiseState.standstill = False
    else:
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["EMS6"]['CRUISE_LAMP_S'])
    #ret.cruiseState.nonAdaptive = cp.vl["PCM_CRUISE"]['CRUISE_STATE'] in [1, 2, 3, 4, 5, 6]
    ret.cruiseState.nonAdaptive = False

    if self.CP.carFingerprint == CAR.PRIUS:
      ret.genericToggle = cp.vl["AUTOPARK_STATUS"]['STATE'] != 0
    else:
      ret.genericToggle = bool(cp_cam.vl["LIGHT_STALK"]['AUTO_HIGH_BEAM'])
    ret.stockAeb = bool(cp_cam.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_cam.vl["PRE_COLLISION"]["FORCE"] < -1e-5)

    ret.espDisabled = False   # Can't disable ESP in i30 (maybe has errorbit)
    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    self.steer_state = cp_cam.vl["EPS_STATUS"]['LKA_STATE']

    ret.epsDisabled = (True if ret.genericToggle == 0 else False)

    if self.CP.carFingerprint in TSS2_CAR:
      ret.leftBlindspot = (cp.vl["BSM"]['L_ADJACENT'] == 1) or (cp.vl["BSM"]['L_APPROACHING'] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]['R_ADJACENT'] == 1) or (cp.vl["BSM"]['R_APPROACHING'] == 1)

    return ret

  @staticmethod
  def get_can_parser(CP):

    signals = [
      # sig_name, sig_address, default
      ("SAS_Angle", "SAS1", 0),               #Imported from i30
      ("SAS_Speed", "SAS1", 0),               #Imported from i30
      ("PV_AV_CAN", "EMS_DCT1", 0),           #Imported from i30
      ("CF_Ems_AclAct", "EMS6", 1),           #Imported from i30
      ("CR_Mdps_StrTq", "VSM2", 0),           #Imported from i30
      ("CR_Mdps_OutTq", "VSM2", 0),           #Imported from i30

      ("WHEEL_FL", "TCS5", 0),                #Imported from i30
      ("WHEEL_FR", "TCS5", 0),                #Imported from i30
      ("WHEEL_RL", "TCS5", 0),                #Imported from i30
      ("WHEEL_RR", "TCS5", 0),                #Imported from i30
      ("CF_Clu_DrvDrSw", "CLU2", 1),          #Imported from i30
      ("CF_Clu_AstDrSw", "CLU2", 1),          #Imported from i30
      ("CF_Clu_DrvSeatBeltSw", "CLU2", 0),   #Imported from i30
      ("CF_Clu_TurnSigLh", "CLU2", 0),        #Imported from i30
      ("CF_Clu_TurnSigRh", "CLU2", 0),        #Imported from i30

      ("CRUISE_LAMP_M", "EMS6", 0),           #Imported from i30
      ("CRUISE_LAMP_S", "EMS6", 0),           #Imported from i30
      ("BRAKE_ACT", "EMS_DCT2", 1),           #Imported from i30
      ("BRAKE_ACT", "EMS2", 1),               #Imported from i30

    ]

    checks = [			# TODO: change the interval to correct when want to do some polishing
        ("EMS_DCT2", 20),	# True interval 10 ms
        ("VSM2", 20),		# True interval 10 ms
        ("TCS5", 20),		# True interval 20 ms
        ("SAS1", 20)		# True interval 10 ms
    ]

    if CP.carFingerprint == CAR.LEXUS_IS:
      signals.append(("MAIN_ON", "DSU_CRUISE", 0))
      signals.append(("SET_SPEED", "DSU_CRUISE", 0))
      checks.append(("DSU_CRUISE", 5))
#    else:
#      signals.append(("MAIN_ON", "PCM_CRUISE_2", 0))
#      signals.append(("SET_SPEED", "PCM_CRUISE_2", 0))
#      signals.append(("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0))
#      checks.append(("PCM_CRUISE_2", 33))

    if CP.carFingerprint == CAR.PRIUS:
      signals += [("STATE", "AUTOPARK_STATUS", 0)]
    if CP.hasZss:
      signals += [("ZORRO_STEER", "SECONDARY_STEER_ANGLE", 0)]

    # add gas interceptor reading if we are using it
#    if CP.enableGasInterceptor:
#      signals.append(("INTERCEPTOR_GAS", "GAS_SENSOR", 0))
#      signals.append(("INTERCEPTOR_GAS2", "GAS_SENSOR", 0))
#      checks.append(("GAS_SENSOR", 50))

    if CP.carFingerprint in TSS2_CAR:
      signals += [("L_ADJACENT", "BSM", 0)]
      signals += [("L_APPROACHING", "BSM", 0)]
      signals += [("R_ADJACENT", "BSM", 0)]
      signals += [("R_APPROACHING", "BSM", 0)]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)  #'pt' is the first dbc file given in values.py dbc_dict (see also init.py from car folder)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = [
      ("FORCE", "PRE_COLLISION", 0),
      ("PRECOLLISION_ACTIVE", "PRE_COLLISION", 0),
      ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
      ("STEERING_TORQUE", "STEERING_STATUS", 0),
      ("STEERING_ANGLE", "STEERING_STATUS", 0),
      ("STEER_ANGLE", "STEER_TORQUE_SENSOR", 0),
      ("AUTO_HIGH_BEAM", "LIGHT_STALK", 0),
      ("ACCEL_CMD", "ACC_CONTROL", 0),
      ("LKA_STATE", "EPS_STATUS", 0),
      ("CRUISE_ACTIVE", "PCM_CRUISE", 0),
      ("CRUISE_STATE", "PCM_CRUISE", 0),
      ("BRK_ST_OP", "PCM_CRUISE", 0),

    ]

    # use STEERING_STATUS message to check if panda is connected to SSC
    checks = [
      #("STEERING_STATUS", 42)    # Try at first no checks if SSC is not connected yet
    ]

    return CANParser('toyota_corolla_2017_pt_generated', signals, checks, 2)		# Use corolla DBC to recognize the used signals
