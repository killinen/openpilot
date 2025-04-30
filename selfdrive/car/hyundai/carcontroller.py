from cereal import car
# from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
# from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
# from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfahda_mfc, create_acc_commands, create_acc_opt, create_frt_radar_opt, create_steer_command
from selfdrive.car.hyundai.hyundaican import create_steer_command
# from selfdrive.car.hyundai.values import Buttons, CarControllerParams, CAR, SteerLimitParams
from selfdrive.car.hyundai.values import CarControllerParams, SteerLimitParams
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

SAMPLING_FREQ = 100 #Hz

# Steer angle limits
ANGLE_MAX_BP = [5., 15., 30]  #m/s (8, 54, 108 km/h)
ANGLE_MAX = [200., 30., 15.] #deg
#ANGLE_MAX = [200., 20., 10.] #deg   (dzids original)
ANGLE_RATE_BP = [0., 5., 15.]
ANGLE_RATE_WINDUP = [500., 80., 15.]     #deg/s windup rate limit
ANGLE_RATE_UNWIND = [500., 350., 40.]  #deg/s unwind rate limit

# def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
#                       right_lane, left_lane_depart, right_lane_depart):
#   sys_warning = (visual_alert in [VisualAlert.steerRequired, VisualAlert.ldw])
#
#   # initialize to no line visible
#   sys_state = 1
#   if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
#     sys_state = 3 if enabled or sys_warning else 4
#   elif left_lane:
#     sys_state = 5
#   elif right_lane:
#     sys_state = 6
#
#   # initialize to no warnings
#   left_lane_warning = 0
#   right_lane_warning = 0
#   if left_lane_depart:
#     left_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
#   if right_lane_depart:
#     right_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
#
#   return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.p = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)

    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.last_resume_frame = 0
    self.accel = 0

    # StepperServo variables, redundant safety check with the board
    self.last_target_angle_lim = 0
    self.last_fault_frame = -200
    self.target_angle_delta = 0

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed,
             left_lane, right_lane, left_lane_depart, right_lane_depart):
    can_sends = []  # Initialize can_sends here

    # Steering Torque
    new_steer = int(round(actuators.steer * self.p.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    # disable when temp fault is active, or below LKA minimum speed
    lkas_active = enabled and not CS.out.steerWarning and CS.out.vEgo >= CS.CP.minSteerSpeed

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

# #####################################################################################################
# ######################################### New Steer Logik ###########################################
# #####################################################################################################

    if not enabled or (frame - self.last_fault_frame < 200):
    #if not enabled or abs(CS.out.steeringRateDeg) > 100:
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    # Cut steering for 2s after fault
    steer_tq = 0
    # steer angle
    angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX)
    target_angle_lim = clip(actuators.steeringAngleDeg, -angle_lim, angle_lim)
    if enabled:
      # windup slower
      if (self.last_target_angle_lim * target_angle_lim) > 0. and abs(target_angle_lim) > abs(self.last_target_angle_lim): #todo revise last_angle
        angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_WINDUP)
      else:
        angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_UNWIND)
      # steer angle - don't allow too large delta
      MAX_SEC_BEHIND = 1 #seconds behind target. Target deltas behind more than 1s will be rejected by bmw_safety #todo implement real (speed) rate limiter?? check with panda. Replace MAX_SEC_BEHIND with a Hz?
      target_angle_lim = clip(target_angle_lim, self.last_target_angle_lim - angle_rate_max*MAX_SEC_BEHIND, self.last_target_angle_lim + angle_rate_max*MAX_SEC_BEHIND)
      self.target_angle_delta =  target_angle_lim - CS.out.steeringAngleDeg
      angle_step_max = angle_rate_max / SAMPLING_FREQ  #max angle step per single sample
      angle_step = clip(self.target_angle_delta, -angle_step_max, angle_step_max) #apply angle step
      self.steer_rate_limited = self.target_angle_delta != angle_step #advertise steer beeing rate limited
      # steer torque
      steer_tq = actuators.steer * 9
      # explicitly clip torque before sending on CAN
      steer_tq = clip(steer_tq, -SteerLimitParams.MAX_STEERING_TQ, SteerLimitParams.MAX_STEERING_TQ)
      # if (frame % 100) == 0: #slow print when disabled
      #   print("SteerAngle {0} SteerSpeed {1}".format(CS.out.steeringAngleDeg,
                                                                #  CS.out.steeringRateDeg))
    # if (frame % 10) == 0:
    #   print(f'offset: SAS angle: {CS.out.steeringAngleDeg}, SSC angle: {CS.out.steeringAngleDegSSC}, steeringAngleDegError: {CS.out.steeringAngleDegError}')

    can_sends.append(create_steer_command(self.packer, apply_steer_req, self.target_angle_delta, steer_tq, frame))
    # can_sends.append(create_steer_command(apply_steer_req, self.target_angle_delta, self.steer_tq_r, frame))

# ###################################################################################################################
# ########################################## End of new Steer Logik #################################################
# ###################################################################################################################

    #sys_warning, sys_state, left_lane_warning, right_lane_warning = \
    #  process_hud_alert(enabled, self.car_fingerprint, visual_alert,
    #                    left_lane, right_lane, left_lane_depart, right_lane_depart)

    # can_sends = []

    # tester present - w/ no response (keeps radar disabled)
    if CS.CP.openpilotLongitudinalControl:
      if (frame % 100) == 0:
        can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    # can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
    #                                CS.lkas11, sys_warning, sys_state, enabled,
    #                                left_lane, right_lane,
    #                                left_lane_warning, right_lane_warning))

    #if not CS.CP.openpilotLongitudinalControl:
    #  if pcm_cancel_cmd:
    #    can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))
    #  elif CS.out.cruiseState.standstill:
    #    # send resume at a max freq of 10Hz
    #    if (frame - self.last_resume_frame) * DT_CTRL > 0.1:
    #      # send 25 messages at a time to increases the likelihood of resume being accepted
    #      can_sends.extend([create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL)] * 25)
    #      self.last_resume_frame = frame

    if frame % 2 == 0 and CS.CP.openpilotLongitudinalControl:
      # lead_visible = False
      accel = actuators.accel if enabled else 0

      # jerk = clip(2.0 * (accel - CS.out.aEgo), -12.7, 12.7)

      if accel < 0:
        accel = interp(accel - CS.out.aEgo, [-1.0, -0.5], [2 * accel, accel])

      accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      # stopping = (actuators.longControlState == LongCtrlState.stopping)
      # set_speed_in_units = hud_speed * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
      # can_sends.extend(create_acc_commands(self.packer, enabled, accel, jerk, int(frame / 2), lead_visible, set_speed_in_units, stopping))
      self.accel = accel

    # 20 Hz LFA MFA message
   # if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE, CAR.IONIQ, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021,
   #                                                CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.KIA_CEED, CAR.KIA_SELTOS, CAR.KONA_EV,
   #                                                CAR.ELANTRA_2021, CAR.ELANTRA_HEV_2021, CAR.SONATA_HYBRID, CAR.KONA_HEV, CAR.SANTA_FE_2022,
   #                                                CAR.KIA_K5_2021, CAR.IONIQ_HEV_2022, CAR.SANTA_FE_HEV_2022, CAR.GENESIS_G70_2020, CAR.SANTA_FE_PHEV_2022]:
   #   can_sends.append(create_lfahda_mfc(self.packer, enabled))

    # 5 Hz ACC options
   # if frame % 20 == 0 and CS.CP.openpilotLongitudinalControl:
   #   can_sends.extend(create_acc_opt(self.packer))

    # 2 Hz front radar options
   # if frame % 50 == 0 and CS.CP.openpilotLongitudinalControl:
   #   can_sends.append(create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.p.STEER_MAX
    new_actuators.accel = self.accel

    return new_actuators, can_sends
