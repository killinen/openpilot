import copy
from collections import deque

from cereal import car, custom
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import mean
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.toyota.values import ToyotaFlags, ToyotaFrogPilotFlags, CAR, DBC, STEER_THRESHOLD, NO_STOP_TIMER_CAR, \
                                                  TSS2_CAR, RADAR_ACC_CAR, EPS_SCALE, UNSUPPORTED_DSU_CAR, SECOC_CAR

SteerControlType = car.CarParams.SteerControlType

LS600H_STEERING_RATE_WINDOW_NS = int(0.25 * 1e9)
LS600H_STEERING_RATE_RESET_NS = int(0.5 * 1e9)
LS600H_HRR_STATUS_MAX_AGE_FRAMES = round(0.25 / DT_CTRL)

# These steering fault definitions seem to be common across LKA (torque) and LTA (angle):
# - high steer rate fault: goes to 21 or 25 for 1 frame, then 9 for 2 seconds
# - lka/lta msg drop out: goes to 9 then 11 for a combined total of 2 seconds, then 3.
#     if using the other control command, goes directly to 3 after 1.5 seconds
# - initializing: LTA can report 0 as long as STEER_TORQUE_SENSOR->STEER_ANGLE_INITIALIZING is 1,
#     and is a catch-all for LKA
TEMP_STEER_FAULTS = (0, 9, 11, 21, 25)
# - lka/lta msg drop out: 3 (recoverable)
# - prolonged high driver torque: 17 (permanent)
PERM_STEER_FAULTS = (3, 17)


def ls600h_hrr_steering_valid(status, age_frames):
  return bool(
    age_frames <= LS600H_HRR_STATUS_MAX_AGE_FRAMES
    and status["True_Angle_Valid"]
    and status["True_Angle_Initialized"]
    and status["True_Angle_Resolver_Valid"]
    and status["True_Angle_Calibrated"]
    and not status["True_Angle_Wrap_Ambiguous"]
  )


# Traffic signals for Speed Limit Controller - Credit goes to the DragonPilot team!
def calculate_speed_limit(cp_cam, frogpilot_toggles):
  speed_limit_unit = cp_cam.vl["RSA1"]["TSGN1"]
  speed_limit_value = cp_cam.vl["RSA1"]["SPDVAL1"]

  if speed_limit_unit == 1 and not frogpilot_toggles.force_mph_dashboard:
    return speed_limit_value * CV.KPH_TO_MS
  elif speed_limit_unit == 36 or frogpilot_toggles.force_mph_dashboard:
    return speed_limit_value * CV.MPH_TO_MS
  else:
    return 0


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.eps_torque_scale = EPS_SCALE[CP.carFingerprint] / 100.
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    if CP.carFingerprint == CAR.LEXUS_LS600h:
      self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]
    elif CP.flags & ToyotaFlags.SECOC.value:
      self.shifter_values = can_define.dv["GEAR_PACKET_HYBRID"]["GEAR"]
    else:
      self.shifter_values = can_define.dv["GEAR_PACKET"]["GEAR"]

    # On cars with cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]
    # the signal is zeroed to where the steering angle is at start.
    # Need to apply an offset as soon as the steering angle measurements are both received
    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL, initialized=False)

    # 0x25 on LS600h has only a verified 1.5-degree coarse angle. Derive a
    # telemetry rate over a long enough interval to avoid one-frame 120 deg/s
    # quantization spikes at the 80 Hz message rate.
    self.ls600h_steering_history = deque()
    self.ls600h_steering_rate = FirstOrderFilter(0.0, 0.1, 1.0 / 80.0)
    self.ls600h_last_steering_ts = 0
    self.ls600h_hrr_status_age_frames = LS600H_HRR_STATUS_MAX_AGE_FRAMES + 1

    self.prev_distance_button = 0
    self.distance_button = 0

    self.pcm_follow_distance = 0

    self.low_speed_lockout = False
    self.acc_type = 1
    self.lkas_hud = {}
    self.gvc = 0.0
    self.secoc_synchronization = None

    # FrogPilot variables
    self.latActive_previous = False
    self.needs_angle_offset_zss = True

    self.angle_offset_zss = 0
    self.zorro_steer_value = 0

  def update_ls600h(self, cp, cp_body, cp_hrr):
    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()

    # Physical wheel ordering is not yet confirmed; the average used for vEgo is unaffected.
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp_body.vl["LS600H_0B0"]["WHEEL_SPEED_1"],
      cp_body.vl["LS600H_0B0"]["WHEEL_SPEED_2"],
      cp_body.vl["LS600H_0B2"]["WHEEL_SPEED_3"],
      cp_body.vl["LS600H_0B2"]["WHEEL_SPEED_4"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = cp.vl["SPEED"]["SPEED"] * CV.KPH_TO_MS
    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    # LS600h 0x25 only has a verified 12-bit coarse angle. Its bytes 4 and 5
    # are identical in the stationary steering capture, so they are not the
    # conventional Toyota fraction/rate packing.
    fallback_steering_angle = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
    steering_ts = cp.ts_nanos["STEER_ANGLE_SENSOR"]["STEER_ANGLE"]
    if steering_ts > self.ls600h_last_steering_ts:
      if self.ls600h_last_steering_ts and steering_ts - self.ls600h_last_steering_ts > LS600H_STEERING_RATE_RESET_NS:
        self.ls600h_steering_history.clear()
        self.ls600h_steering_rate = FirstOrderFilter(0.0, 0.1, 1.0 / 80.0)

      self.ls600h_steering_history.append((steering_ts, fallback_steering_angle))
      cutoff = steering_ts - LS600H_STEERING_RATE_WINDOW_NS
      while len(self.ls600h_steering_history) > 1 and self.ls600h_steering_history[1][0] <= cutoff:
        self.ls600h_steering_history.popleft()

      if len(self.ls600h_steering_history) > 1 and self.ls600h_steering_history[0][0] <= cutoff:
        start_ts, start_angle = self.ls600h_steering_history[0]
        raw_rate = (fallback_steering_angle - start_angle) / ((steering_ts - start_ts) * 1e-9)
      else:
        raw_rate = 0.0
      self.ls600h_steering_rate.update(raw_rate)
      self.ls600h_last_steering_ts = steering_ts

    ret.steeringAngleDeg = fallback_steering_angle
    ret.steeringRateDeg = self.ls600h_steering_rate.x
    if cp_hrr is not None:
      hrr_status = cp_hrr.vl["HRR_TrueAngleStatus"]
      if cp_hrr.vl_all["HRR_TrueAngleStatus"]["True_Angle_Counter"]:
        self.ls600h_hrr_status_age_frames = 0
      else:
        self.ls600h_hrr_status_age_frames += 1

      if ls600h_hrr_steering_valid(hrr_status, self.ls600h_hrr_status_age_frames):
        ret.steeringAngleDeg = hrr_status["True_Steering_Angle"]
        ret.steeringRateDeg = hrr_status["True_Steering_Rate"]
    ret.yawRate = cp.vl["KINEMATICS"]["YAW_RATE"]

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # The HRR supplies an emulated torque value, not reliable driver-applied torque.
    # Keep it logged for analysis but do not use it to freeze lateral control.
    ret.steeringPressed = False

    ret.brakePressed = cp_body.vl["LS600H_2C6"]["BRAKE_PRESSED"] != 0
    ret.gasPressed = cp_body.vl["LS600H_126"]["GAS_PEDAL_PRESSED"] != 0

    body_state = cp_body.vl["BODY_CONTROL_STATE"]
    ret.doorOpen = any([body_state["DOOR_OPEN_FL"], body_state["DOOR_OPEN_FR"],
                        body_state["DOOR_OPEN_RL"], body_state["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = body_state["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = body_state["PARKING_BRAKE"] == 1

    gear_packet = cp_body.vl["GEAR_PACKET"]
    gear = "B" if gear_packet["B_GEAR_ENGAGED"] else self.shifter_values.get(int(gear_packet["GEAR"]))
    ret.gearShifter = self.parse_gear_shifter(gear)

    cruise_active = bool(cp_body.vl["LS600H_124"]["CRUISE_ACTIVE"])
    ret.cruiseState.available = cruise_active
    ret.cruiseState.enabled = cruise_active

    self.gvc = cp.vl["VSC1S07"]["GVC"]
    # The LS-specific frame currently exposes only active/inactive, not Toyota's full CRUISE_STATE enum.
    self.pcm_acc_status = 8 if cruise_active else 0
    self.cruise_decreased = False
    self.cruise_increased = False
    self.distance_button = 0
    self.lkas_previously_enabled = self.lkas_enabled
    self.lkas_enabled = False

    return ret, fp_ret

  def update(self, cp, cp_cam, CC, frogpilot_toggles, cp_hrr=None):
    if self.CP.carFingerprint == CAR.LEXUS_LS600h:
      return self.update_ls600h(cp, cp_cam, cp_hrr)

    ret = car.CarState.new_message()
    fp_ret = custom.FrogPilotCarState.new_message()
    cp_acc = cp_cam if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) else cp

    if not self.CP.flags & ToyotaFlags.SECOC.value:
      self.gvc = cp.vl["VSC1S07"]["GVC"]

    ret.doorOpen = any([cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_FR"],
                        cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RL"], cp.vl["BODY_CONTROL_STATE"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["BODY_CONTROL_STATE"]["SEATBELT_DRIVER_UNLATCHED"] != 0
    ret.parkingBrake = cp.vl["BODY_CONTROL_STATE"]["PARKING_BRAKE"] == 1

    ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    ret.brakeHoldActive = cp.vl["ESP_CONTROL"]["BRAKE_HOLD_ACTIVE"] == 1

    if self.CP.flags & ToyotaFlags.SECOC.value:
      self.secoc_synchronization = copy.copy(cp.vl["SECOC_SYNCHRONIZATION"])
      ret.gas = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"]
      ret.gasPressed = cp.vl["GAS_PEDAL"]["GAS_PEDAL_USER"] > 0
      can_gear = int(cp.vl["GEAR_PACKET_HYBRID"]["GEAR"])
    else:
      if self.CP.enableGasInterceptor:
        ret.gas = (cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"] + cp.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"]) // 2
        ret.gasPressed = ret.gas > 805
      else:
        ret.gasPressed = cp.vl["PCM_CRUISE"]["GAS_RELEASED"] == 0  # TODO: these also have GAS_PEDAL, come back and unify
      can_gear = int(cp.vl["GEAR_PACKET"]["GEAR"])
      if not self.CP.enableDsu and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
        ret.stockAeb = bool(cp_acc.vl["PRE_COLLISION"]["PRECOLLISION_ACTIVE"] and cp_acc.vl["PRE_COLLISION"]["FORCE"] < -1e-5)
      if self.CP.carFingerprint != CAR.TOYOTA_MIRAI:
        ret.engineRpm = cp.vl["ENGINE_RPM"]["RPM"]

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo * frogpilot_toggles.cluster_offset

    ret.standstill = abs(ret.vEgoRaw) < 1e-3

    ret.steeringAngleDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_ANGLE"] + cp.vl["STEER_ANGLE_SENSOR"]["STEER_FRACTION"]
    ret.steeringRateDeg = cp.vl["STEER_ANGLE_SENSOR"]["STEER_RATE"]
    torque_sensor_angle_deg = cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE"]

    # On some cars, the angle measurement is non-zero while initializing
    if abs(torque_sensor_angle_deg) > 1e-3 and not bool(cp.vl["STEER_TORQUE_SENSOR"]["STEER_ANGLE_INITIALIZING"]):
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      # Offset seems to be invalid for large steering angles and high angle rates
      if abs(ret.steeringAngleDeg) < 90 and abs(ret.steeringRateDeg) < 100 and cp.can_valid:
        self.angle_offset.update(torque_sensor_angle_deg - ret.steeringAngleDeg)

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = torque_sensor_angle_deg - self.angle_offset.x

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["BLINKERS_STATE"]["TURN_SIGNALS"] == 2

    ret.steeringTorque = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["STEER_TORQUE_SENSOR"]["STEER_TORQUE_EPS"] * self.eps_torque_scale
    # we could use the override bit from dbc, but it's triggered at too high torque values
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # Check EPS LKA/LTA fault status
    ret.steerFaultTemporary = cp.vl["EPS_STATUS"]["LKA_STATE"] in TEMP_STEER_FAULTS
    ret.steerFaultPermanent = cp.vl["EPS_STATUS"]["LKA_STATE"] in PERM_STEER_FAULTS

    if self.CP.steerControlType == SteerControlType.angle:
      ret.steerFaultTemporary = ret.steerFaultTemporary or cp.vl["EPS_STATUS"]["LTA_STATE"] in TEMP_STEER_FAULTS
      ret.steerFaultPermanent = ret.steerFaultPermanent or cp.vl["EPS_STATUS"]["LTA_STATE"] in PERM_STEER_FAULTS

    if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      # TODO: find the bit likely in DSU_CRUISE that describes an ACC fault. one may also exist in CLUTCH
      ret.cruiseState.available = cp.vl["DSU_CRUISE"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["DSU_CRUISE"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_ALT"]["UI_SET_SPEED"]
    else:
      ret.accFaulted = cp.vl["PCM_CRUISE_2"]["ACC_FAULTED"] != 0
      ret.cruiseState.available = cp.vl["PCM_CRUISE_2"]["MAIN_ON"] != 0
      ret.cruiseState.speed = cp.vl["PCM_CRUISE_2"]["SET_SPEED"] * CV.KPH_TO_MS
      cluster_set_speed = cp.vl["PCM_CRUISE_SM"]["UI_SET_SPEED"]

    # UI_SET_SPEED is always non-zero when main is on, hide until first enable
    if ret.cruiseState.speed != 0:
      is_metric = cp.vl["BODY_CONTROL_STATE_2"]["UNITS"] in (1, 2)
      conversion_factor = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
      ret.cruiseState.speedCluster = cluster_set_speed * conversion_factor

    if self.CP.carFingerprint in TSS2_CAR and not self.CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if not (self.CP.flags & ToyotaFlags.SMART_DSU.value):
        self.acc_type = cp_acc.vl["ACC_CONTROL"]["ACC_TYPE"]
      ret.stockFcw = bool(cp_acc.vl["PCS_HUD"]["FCW"])

    # some TSS2 cars have low speed lockout permanently set, so ignore on those cars
    # these cars are identified by an ACC_TYPE value of 2.
    # TODO: it is possible to avoid the lockout and gain stop and go if you
    # send your own ACC_CONTROL msg on startup with ACC_TYPE set to 1
    if (self.CP.carFingerprint not in TSS2_CAR and self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR) or \
       (self.CP.carFingerprint in TSS2_CAR and self.acc_type == 1):
      self.low_speed_lockout = cp.vl["PCM_CRUISE_2"]["LOW_SPEED_LOCKOUT"] == 2

    self.pcm_acc_status = cp.vl["PCM_CRUISE"]["CRUISE_STATE"]
    if self.CP.carFingerprint not in (NO_STOP_TIMER_CAR - TSS2_CAR):
      # ignore standstill state in certain vehicles, since pcm allows to restart with just an acceleration request
      ret.cruiseState.standstill = self.pcm_acc_status == 7
    ret.cruiseState.enabled = bool(cp.vl["PCM_CRUISE"]["CRUISE_ACTIVE"])
    ret.cruiseState.nonAdaptive = self.pcm_acc_status in (1, 2, 3, 4, 5, 6)

    ret.genericToggle = bool(cp.vl["LIGHT_STALK"]["AUTO_HIGH_BEAM"])
    ret.espDisabled = cp.vl["ESP_CONTROL"]["TC_DISABLED"] != 0

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSM"]["L_ADJACENT"] == 1) or (cp.vl["BSM"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSM"]["R_ADJACENT"] == 1) or (cp.vl["BSM"]["R_APPROACHING"] == 1)

    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      self.lkas_hud = copy.copy(cp_cam.vl["LKAS_HUD"])

    if self.CP.carFingerprint not in UNSUPPORTED_DSU_CAR:
      self.pcm_follow_distance = cp.vl["PCM_CRUISE_2"]["PCM_FOLLOW_DISTANCE"]

    if self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR) or (self.CP.flags & ToyotaFlags.SMART_DSU and not self.CP.flags & ToyotaFlags.RADAR_CAN_FILTER):
      # distance button is wired to the ACC module (camera or radar)
      self.prev_distance_button = self.distance_button
      if self.CP.carFingerprint in (SECOC_CAR - RADAR_ACC_CAR):
        self.distance_button = cp.vl["PCM_CRUISE_4"]["DISTANCE"]
      elif self.CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
        self.distance_button = cp_acc.vl["ACC_CONTROL"]["DISTANCE"]
      else:
        self.distance_button = cp.vl["SDSU"]["FD_BUTTON"]

    # FrogPilot CarState functions
    fp_ret.brakeLights = bool(cp.vl["ESP_CONTROL"]["BRAKE_LIGHTS_ACC"])

    self.cruise_decreased = self.pcm_acc_status == 10
    self.cruise_increased = self.pcm_acc_status == 9

    fp_ret.dashboardSpeedLimit = calculate_speed_limit(cp_cam, frogpilot_toggles)

    if not self.CP.flags & ToyotaFlags.SECOC.value:
      fp_ret.ecoGear = cp.vl["GEAR_PACKET"]["ECON_ON"] == 1
      fp_ret.sportGear = cp.vl["GEAR_PACKET"]["SPORT_ON_2" if self.CP.flags & ToyotaFlags.NO_DSU else "SPORT_ON"] == 1

    self.lkas_previously_enabled = self.lkas_enabled
    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      self.lkas_enabled = self.lkas_hud.get("LDA_ON_MESSAGE") == 1

    # ZSS Support - Credit goes to Erich!
    if self.FPCP.fpFlags & ToyotaFrogPilotFlags.ZSS:
      if abs(torque_sensor_angle_deg) > 1e-3:
        self.accurate_steer_angle_seen = True

      if CC.latActive and not self.latActive_previous:
        self.needs_angle_offset_zss = True
      self.latActive_previous = CC.latActive

      if self.needs_angle_offset_zss:
        zorro_steer = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"]
        if abs(ret.steeringAngleDeg) > 1e-3 and abs(zorro_steer) > 1e-3:
          self.needs_angle_offset_zss = False
          self.angle_offset_zss = zorro_steer - ret.steeringAngleDeg
      self.zorro_steer_value = cp.vl["SECONDARY_STEER_ANGLE"]["ZORRO_STEER"] - self.angle_offset_zss

      if not self.needs_angle_offset_zss:
        if abs(ret.steeringAngleDeg - self.zorro_steer_value) > 4.0:
          ret.steeringAngleDeg = ret.steeringAngleDeg
        else:
          ret.steeringAngleDeg = self.zorro_steer_value

    return ret, fp_ret

  @staticmethod
  def get_can_parser(CP, FPCP):
    if CP.carFingerprint == CAR.LEXUS_LS600h:
      messages = [
        ("KINEMATICS", 80),
        ("SPEED", 40),
        ("STEER_ANGLE_SENSOR", 80),
        ("STEER_TORQUE_SENSOR", 50),
        ("VSC1S07", 20),
      ]
      return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

    messages = [
      ("LIGHT_STALK", 1),
      ("BLINKERS_STATE", 0.15),
      ("BODY_CONTROL_STATE", 3),
      ("BODY_CONTROL_STATE_2", 2),
      ("ESP_CONTROL", 3),
      ("EPS_STATUS", 25),
      ("BRAKE_MODULE", 40),
      ("WHEEL_SPEEDS", 80),
      ("STEER_ANGLE_SENSOR", 80),
      ("PCM_CRUISE", 33),
      ("PCM_CRUISE_SM", 1),
      ("STEER_TORQUE_SENSOR", 50),
    ]

    if CP.flags & ToyotaFlags.SECOC.value:
      messages += [
        ("GEAR_PACKET_HYBRID", 60),
        ("SECOC_SYNCHRONIZATION", 10),
        ("GAS_PEDAL", 42),
      ]
      if CP.carFingerprint not in RADAR_ACC_CAR:
        messages.append(("PCM_CRUISE_4", 1))
    else:
      messages.append(("VSC1S07", 20))
      if CP.carFingerprint not in [CAR.TOYOTA_MIRAI]:
        messages.append(("ENGINE_RPM", 42))

      messages += [
        ("GEAR_PACKET", 1),
      ]

    if CP.carFingerprint in UNSUPPORTED_DSU_CAR:
      messages.append(("DSU_CRUISE", 5))
      messages.append(("PCM_CRUISE_ALT", 1))
    else:
      messages.append(("PCM_CRUISE_2", 33))

    # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:
      messages.append(("GAS_SENSOR", 50))

    if CP.enableBsm:
      messages.append(("BSM", 1))

    if CP.carFingerprint in RADAR_ACC_CAR and not CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      if not CP.flags & ToyotaFlags.SMART_DSU.value:
        messages += [
          ("ACC_CONTROL", 33),
        ]
      messages += [
        ("PCS_HUD", 1),
      ]

    if CP.carFingerprint not in (TSS2_CAR - RADAR_ACC_CAR) and not CP.enableDsu and not CP.flags & ToyotaFlags.DISABLE_RADAR.value:
      messages += [
        ("PRE_COLLISION", 33),
      ]

    if CP.flags & ToyotaFlags.SMART_DSU and not CP.flags & ToyotaFlags.RADAR_CAN_FILTER:
      messages += [
        ("SDSU", 100),
      ]

    if FPCP.fpFlags & ToyotaFrogPilotFlags.ZSS:
      messages += [("SECONDARY_STEER_ANGLE", 0)]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP, FPCP):
    if CP.carFingerprint == CAR.LEXUS_LS600h:
      messages = [
        ("LS600H_0B0", 10),
        ("LS600H_0B2", 10),
        ("LS600H_124", 5),
        ("LS600H_126", 10),
        ("LS600H_2C6", 5),
        ("GEAR_PACKET", 0.1),
        ("BODY_CONTROL_STATE", 0.5),
      ]
      return CANParser(DBC[CP.carFingerprint]["pt"], messages, 1)

    messages = []

    messages += [
      ("RSA1", 0),
      ("RSA2", 0),
    ]

    if CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      messages += [
        ("LKAS_HUD", 1),
      ]

    if CP.carFingerprint in (TSS2_CAR - RADAR_ACC_CAR):
      messages += [
        ("ACC_CONTROL", 33),
        ("PCS_HUD", 1),
      ]

      # TODO: Figure out new layout of the PRE_COLLISION message
      if not CP.flags & ToyotaFlags.SECOC.value:
        messages += [
          ("PRE_COLLISION", 33),
        ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)

  @staticmethod
  def get_body_can_parser(CP):
    if CP.carFingerprint == CAR.LEXUS_LS600h:
      # HRR is optional for state estimation: frequency 0 prevents a missing
      # controller from invalidating the vehicle CAN parsers. CarState applies
      # its own freshness and status-flag checks before using this measurement.
      return CANParser(DBC[CP.carFingerprint]["pt"], [("HRR_TrueAngleStatus", 0)], 2)
    return None
