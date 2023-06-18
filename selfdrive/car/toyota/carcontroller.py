from cereal import car, messaging
from common.numpy_fast import clip, interp
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_interceptor_command, make_can_msg
from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, \
                                           create_accel_command, create_acc_cancel_command, \
                                           create_fcw_command, create_lta_steer_command, \
                                           create_lead_command, create_new_steer_command
from selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, \
                                        SteerLimitParams
from opendbc.can.packer import CANPacker
from common.op_params import opParams

VisualAlert = car.CarControl.HUDControl.VisualAlert

def calc_steering_torque_hold(angle, vEgo):
  hold_BP = [-40.0, -6.0, -4.0, -3.0, -2.0, -1.0, -0.5,  0.5,  1.0,  2.0,  3.0,  4.0,  6.0, 40.0]
  hold_V  = [-12.0, -5.7, -5.0, -4.5, -4.0, -3.3, -2.5,  2.5,  3.3,  4.0,  4.5,  5.0,  5.7, 12.0]
  return interp(angle, hold_BP, hold_V) #todo substract angle offset

SAMPLING_FREQ = 100 #Hz

# Steer angle limits
ANGLE_MAX_BP = [5., 15., 30]  #m/s
ANGLE_MAX = [200., 40., 20.] #deg
#ANGLE_MAX = [200., 20., 10.] #deg   (dzids origigal)
ANGLE_RATE_BP = [0., 5., 15.]
ANGLE_RATE_WINDUP = [500., 80., 15.]     #deg/s windup rate limit
ANGLE_RATE_UNWIND = [500., 350., 40.]  #deg/s unwind rate limit

def compute_gb_pedal(accel, speed):
  _a3, _a4, _a5, _offset, _e1, _e2, _e3, _e4, _e5, _e6, _e7, _e8 = [-0.07264304340456754, -0.007522016704006004, 0.16234124452228196, 0.0029096574419830296, 1.1674372321165579e-05, -0.008010070095545522, -5.834025253616562e-05, 0.04722441060805912, 0.001887454016549489, -0.0014370672920621269, -0.007577594283906699, 0.01943515032956308]
  speed_part = (_e5 * accel + _e6) * speed ** 2 + (_e7 * accel + _e8) * speed
  accel_part = ((_e1 * speed + _e2) * accel ** 5 + (_e3 * speed + _e4) * accel ** 4 + _a3 * accel ** 3 + _a4 * accel ** 2 + _a5 * accel)
  return speed_part + accel_part + _offset


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_limited = False
    self.standstill_hack = opParams().get('standstill_hack')
    
    # StepperServo variables, redundant safety check with the board
    self.last_steer_tq = 0
    self.last_controls_enabled = False
    self.last_target_angle_lim = 0
    self.angle_control = False
    self.steer_angle_enabled = False
    self.last_fault_frame = -200
    self.planner_cnt = 0
    self.inertia_tq = 0.
    self.target_angle_delta = 0
    self.steer_tq_r = 0

    self.packer = CANPacker(dbc_name)
    self.gas = 0
    self.accel = 0

    self.lead_v = 100
    self.lead_a = 0
    self.lead_d = 250
    self.sm = messaging.SubMaster(['radarState'])

  def update(self, enabled, active, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):

    # Update lead car stats to be streamed on CAN
    self.sm.update(0)
    if self.sm.updated['radarState']:
      self.lead_v = self.sm['radarState'].leadOne.vRel
      self.lead_a = self.sm['radarState'].leadOne.aRel
      self.lead_d = self.sm['radarState'].leadOne.dRel

    # gas and brake
    if CS.CP.enableGasInterceptor and enabled:
      MAX_INTERCEPTOR_GAS = 0.5
      # RAV4 has very sensitive gas pedal
      if CS.CP.carFingerprint in [CAR.RAV4, CAR.RAV4H, CAR.HIGHLANDER, CAR.HIGHLANDERH]:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif CS.CP.carFingerprint in [CAR.COROLLA]:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # offset for creep and windbrake
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    else:
      interceptor_gas_cmd = 0.
    pcm_accel_cmd = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    # steer torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, CarControllerParams)
    self.steer_rate_limited = new_steer != apply_steer

    # Cut steering while we're in a known fault state (2s) or epsDisabled is set to 1
    # if not enabled or CS.steer_state in [9, 25] or CS.out.epsDisabled==1 or abs(CS.out.steeringRateDeg) > 100:
    if not enabled or abs(CS.out.steeringRateDeg) > 100:        # E39 StepperServoCAN LaC
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    # TODO: probably can delete this. CS.pcm_acc_status uses a different signal
    # than CS.cruiseState.enabled. confirm they're not meaningfully different
    if not enabled and CS.pcm_acc_status:
      pcm_cancel_cmd = 1

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR and not self.standstill_hack:
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_steer = apply_steer
    self.last_standstill = CS.out.standstill

    can_sends = []

    # Send LEAD_INFO CAN msg
    if (frame%2==0):
      can_sends.append(create_lead_command(self.packer, self.lead_v, self.lead_a, self.lead_d))

      
      
# ############################# New Steer Logik ####################################

    # Cut steering for 2s after fault
    steer_tq = 0
    #if not enabled or (frame - self.last_fault_frame < 200):   # I don't think I have last_fault_frame, use old statement below
    # This is done allready above
    # if not enabled or abs(CS.out.steeringRateDeg) > 100:
    #    apply_steer_req = 0
    # else:
    #   apply_steer_req = 1

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
      I_steering = 0 #estimated moment of inertia
      
      PLANNER_SAMPLING_SUBRATE = 6 #planner updates target angle every 4 or 6 samples
      if target_angle_lim != self.last_target_angle_lim or self.planner_cnt >= PLANNER_SAMPLING_SUBRATE-1:
        steer_acc = (target_angle_lim - self.last_target_angle_lim) * SAMPLING_FREQ  #desired acceleration
        remaining_steer_torque = self.inertia_tq * (PLANNER_SAMPLING_SUBRATE - self.planner_cnt -1) #remaining torque to be applied if target_angle_lim was updated earlier than PLANNER_SAMPLING_SUBRATE
        self.inertia_tq = I_steering * steer_acc / PLANNER_SAMPLING_SUBRATE * CV.DEG_TO_RAD  #kg*m^2 * rad/s^2 = N*m (torque)
        self.inertia_tq += remaining_steer_torque / PLANNER_SAMPLING_SUBRATE
        self.planner_cnt = 0
      else:
        self.planner_cnt += 1
      
      # add feed-forward and inertia compensation
      feedforward = calc_steering_torque_hold(target_angle_lim, CS.out.vEgo)
      steer_tq = feedforward + actuators.steer + self.inertia_tq
      # explicitly clip torque before sending on CAN
      steer_tq = clip(steer_tq, -SteerLimitParams.MAX_STEERING_TQ, SteerLimitParams.MAX_STEERING_TQ)
      self.steer_tq_r = steer_tq * (-1)    # Switch StepperServo rotation
      
      # can_sends.append(create_new_steer_command(self.packer, apply_steer_req, self.target_angle_delta, self.steer_tq_r, frame))
      # *** control msgs ***
      # if (frame % 10) == 0: #slow print
      #   print("SteerAngle {0} Inertia  {1} Brake {2}, frame {3}".format(target_angle_lim,
      #                                                            self.inertia_tq,
      #                                                            actuators.brake, speed_diff_req))
    elif not enabled and self.last_controls_enabled: #falling edge - send cancel CAN message
      self.target_angle_delta = 0
      steer_tq = 0
      self.steer_tq_r = 0
      # can_sends.append(create_new_steer_command(self.packer, apply_steer_req, self.target_angle_delta, self.steer_tq_r, frame)) 
      
      # if (frame % 100) == 0: #slow print when disabled
      #   print("SteerAngle {0} SteerSpeed {1}".format(CS.out.steeringAngleDeg,
                                                                #  CS.out.steeringRateDeg))
#     self.last_target_angle_lim = target_angle_lim
  
      self.last_steer_tq = steer_tq
      self.last_target_angle_lim = target_angle_lim
      # self.last_accel = apply_accel
      # self.last_standstill = CS.out.standstill
      self.last_controls_enabled = enabled
  
  
# ########################################## End of new Steer Logik #################################################
      
      
    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))
    # StepperServoCan steer command to CAN
    can_sends.append(create_new_steer_command(self.packer, apply_steer_req, self.target_angle_delta, self.steer_tq_r, frame))
    if frame % 2 == 0 and CS.CP.carFingerprint in TSS2_CAR:
      can_sends.append(create_lta_steer_command(self.packer, 0, 0, frame // 2))

    # LTA mode. Set ret.steerControlType = car.CarParams.SteerControlType.angle and whitelist 0x191 in the panda
    # if frame % 2 == 0:
    #   can_sends.append(create_steer_command(self.packer, 0, 0, frame // 2))
    #   can_sends.append(create_lta_steer_command(self.packer, actuators.steeringAngleDeg, apply_steer_req, frame // 2))

    # we can spam can to cancel the system even if we are using lat only control
    if (frame % 3 == 0 and CS.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = lead or CS.out.vEgo < 12.    # at low speed we always assume the lead is present so ACC can be engaged

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and CS.CP.carFingerprint in [CAR.LEXUS_IS, CAR.LEXUS_RC]:
        can_sends.append(create_acc_cancel_command(self.packer))
      elif CS.CP.openpilotLongitudinalControl:
        can_sends.append(create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type, CS.distance_btn))
        self.accel = pcm_accel_cmd
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead, CS.acc_type, CS.distance_btn))

    if frame % 2 == 0 and CS.CP.enableGasInterceptor and CS.CP.openpilotLongitudinalControl:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, frame // 2))
      self.gas = interceptor_gas_cmd

    # ui mesg is at 100Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    fcw_alert = hud_alert == VisualAlert.fcw
    steer_alert = hud_alert in [VisualAlert.steerRequired, VisualAlert.ldw]

    send_ui = False
    if ((fcw_alert or steer_alert) and not self.alert_active) or \
       (not (fcw_alert or steer_alert) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    elif pcm_cancel_cmd:
      # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
      send_ui = True

    if (frame % 100 == 0 or send_ui):
      can_sends.append(create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, left_line, right_line, left_lane_depart, right_lane_depart, enabled))

    if frame % 100 == 0 and CS.CP.enableDsu:
      can_sends.append(create_fcw_command(self.packer, fcw_alert))

    # *** static msgs ***
    for (addr, cars, bus, fr_step, vl) in STATIC_DSU_MSGS:
      if frame % fr_step == 0 and CS.CP.enableDsu and CS.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    return new_actuators, can_sends
