from cereal import car
# from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai import hda2can, hyundaican
from selfdrive.car.hyundai.values import Buttons, CarControllerParams, HDA2_CAR, CAR, SteerLimitParams

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

# Simple exponential smoothing
def lowpass_filter(new_val, prev_val, alpha):
  return alpha * new_val + (1 - alpha) * prev_val

# Use modded torque limiter from selfdrive/car/__init__.py
def apply_ssc_steer_torque_limits(apply_torque, apply_torque_last, LIMITS):
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque,
                        max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque,
                        apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  apply_torque = clip(apply_torque, -LIMITS.MAX_STEERING_TQ, LIMITS.MAX_STEERING_TQ)

  return apply_torque


# def process_hud_alert(enabled, fingerprint, hud_control):
#   sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

#   # initialize to no line visible
#   sys_state = 1
#   if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
#     sys_state = 3 if enabled or sys_warning else 4
#   elif hud_control.leftLaneVisible:
#     sys_state = 5
#   elif hud_control.rightLaneVisible:
#     sys_state = 6

#   # initialize to no warnings
#   left_lane_warning = 0
#   right_lane_warning = 0
#   if hud_control.leftLaneDepart:
#     left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
#   if hud_control.rightLaneDepart:
#     right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

#   return sys_warning, sys_state, left_lane_warning, right_lane_warning

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.steer_rate_limited = False
    self.last_button_frame = 0
    self.accel = 0

    # StepperServo variables, redundant safety check with the board
    self.last_target_angle_lim = 0
    self.last_fault_frame = -200
    self.target_angle_delta = 0
    self.last_steer_tq = 0

  def update(self, CC, CS):
    actuators = CC.actuators
    # hud_control = CC.hudControl

    # Steering Torque

    # These cars have significantly more torque than most HKG.  Limit to 70% of max.
    steer = actuators.steer
    if self.CP.carFingerprint in (CAR.KONA, CAR.KONA_EV, CAR.KONA_HEV):
      steer = clip(steer, -0.7, 0.7)
    new_steer = int(round(steer * self.params.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
    self.steer_rate_limited = new_steer != apply_steer

    if not CC.latActive:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    # sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
    #                                                                                   hud_control)

# #####################################################################################################
# ######################################### New Steer Logik ###########################################
# #####################################################################################################

    # latActive is when OP latControl is ON
    if not CC.latActive:
    #if not enabled or abs(CS.out.steeringRateDeg) > 100:
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    # Cut steering for 2s after fault
    steer_tq = 0
    # steer angle
    angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX)
    target_angle_lim = clip(actuators.steeringAngleDeg, -angle_lim, angle_lim)
    # CC.enabled is when cruise control is ON but does not mean that it neccassarily is active
    if CC.enabled:
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
      raw_steer_tq = actuators.steer * SteerLimitParams.STEER_MAX
      # explicitly clip torque before sending on CAN -> This was moved to apply_scc_steer_torque_limits()
      #raw_steer_tq = clip(raw_steer_tq, -SteerLimitParams.MAX_STEERING_TQ, SteerLimitParams.MAX_STEERING_TQ)

      # Filter the output to reduce actuator jitter
      alpha = 0.35  # Lower = smoother but more lag

      # First order low pass filter
      raw_steer_tq = lowpass_filter(raw_steer_tq, self.last_steer_tq, alpha)
      # self.last_steer_tq = steer_tq

      # Apply steering torque derivate limits
      steer_tq = apply_ssc_steer_torque_limits(raw_steer_tq, self.last_steer_tq, SteerLimitParams)
      #steer_tq = raw_steer_tq

      self.last_steer_tq = steer_tq

      # if (self.frame % 100) == 0: #slow print when disabled
      #   print("SteerAngle {0} SteerSpeed {1}".format(CS.out.steeringAngleDeg,
                                                                #  CS.out.steeringRateDeg))
      # if (self.frame % 10) == 0:
      #   print(f'apply_steer_rq: {apply_steer_req}, steer_tq: {steer_tq}')
    #   print(f'offset: SAS angle: {CS.out.steeringAngleDeg}, SSC angle: {CS.out.steeringAngleDegSSC}, steeringAngleDegError: {CS.out.steeringAngleDegError}')

    # can_sends.append(hyundaican.create_steer_command(self.packer, apply_steer_req, self.target_angle_delta, steer_tq, frame))
    # can_sends.append(create_steer_command(apply_steer_req, self.target_angle_delta, self.steer_tq_r, frame))

# ###################################################################################################################
# ########################################## End of new Steer Logik #################################################
# ###################################################################################################################

    can_sends = []

    # Send SSC steering command
    can_sends.append(hyundaican.create_steer_command(self.packer, apply_steer_req, self.target_angle_delta, steer_tq, self.frame))

    if self.CP.carFingerprint in HDA2_CAR:
      # steering control
      can_sends.append(hda2can.create_lkas(self.packer, CC.enabled, self.frame, CC.latActive, apply_steer))

      if self.frame % 5 == 0:
        can_sends.append(hda2can.create_cam_0x2a4(self.packer, self.frame, CS.cam_0x2a4))

      # cruise cancel
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        if CC.cruiseControl.cancel:
          for _ in range(20):
            can_sends.append(hda2can.create_buttons(self.packer, CS.buttons_counter+1, Buttons.CANCEL))
          self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          can_sends.append(hda2can.create_buttons(self.packer, CS.buttons_counter+1, Buttons.RES_ACCEL))
          self.last_button_frame = self.frame
    else:

      # tester present - w/ no response (keeps radar disabled)
      if self.CP.openpilotLongitudinalControl:
        if self.frame % 100 == 0:
          can_sends.append([0x7D0, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", 0])

    #   can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, CC.latActive,
    #                                  CS.lkas11, sys_warning, sys_state, CC.enabled,
    #                                  hud_control.leftLaneVisible, hud_control.rightLaneVisible,
    #                                  left_lane_warning, right_lane_warning))

    #   if not self.CP.openpilotLongitudinalControl:
    #     if CC.cruiseControl.cancel:
    #       can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL))
    #     elif CC.cruiseControl.resume:
    #       # send resume at a max freq of 10Hz
    #       if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
    #         # send 25 messages at a time to increases the likelihood of resume being accepted
    #         can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL)] * 25)
    #         self.last_button_frame = self.frame

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
        accel = actuators.accel
        # jerk = 0

        if CC.longActive:
          # jerk = clip(2.0 * (accel - CS.out.aEgo), -12.7, 12.7)
          if accel < 0:
            accel = interp(accel - CS.out.aEgo, [-1.0, -0.5], [2 * accel, accel])

        accel = clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

        #stopping = actuators.longControlState == LongCtrlState.stopping
        #set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_MPH if CS.clu11["CF_Clu_SPEED_UNIT"] == 1 else CV.MS_TO_KPH)
        # can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
        #                                                 hud_control.leadVisible, set_speed_in_units, stopping, CS.out.gasPressed))
        self.accel = accel

    #   # 20 Hz LFA MFA message
    #   if self.frame % 5 == 0 and self.car_fingerprint in (CAR.SONATA, CAR.PALISADE, CAR.IONIQ, CAR.KIA_NIRO_EV, CAR.KIA_NIRO_HEV_2021,
    #                                                       CAR.IONIQ_EV_2020, CAR.IONIQ_PHEV, CAR.KIA_CEED, CAR.KIA_SELTOS, CAR.KONA_EV,
    #                                                       CAR.ELANTRA_2021, CAR.ELANTRA_HEV_2021, CAR.SONATA_HYBRID, CAR.KONA_HEV, CAR.SANTA_FE_2022,
    #                                                       CAR.KIA_K5_2021, CAR.IONIQ_HEV_2022, CAR.SANTA_FE_HEV_2022, CAR.GENESIS_G70_2020, CAR.SANTA_FE_PHEV_2022):
    #     can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

    #   # 5 Hz ACC options
    #   if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
    #     can_sends.extend(hyundaican.create_acc_opt(self.packer))

    #   # 2 Hz front radar options
    #   if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
    #     can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
