from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_command, make_can_msg
from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, \
                                           create_accel_command, create_acc_cancel_command, \
                                           create_fcw_command, create_lta_steer_command, create_lead_command, create_new_steer_command
from selfdrive.car.toyota.values import Ecu, CAR, STATIC_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, CarControllerParams, MIN_ACC_SPEED
from opendbc.can.packer import CANPacker
from common.op_params import opParams
import cereal.messaging as messaging

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Steer angle limits
ANGLE_MAX_BP = [5., 15., 30]  #m/s
ANGLE_MAX = [200., 20., 10.] #deg
ANGLE_RATE_BP = [0., 5., 15.]
ANGLE_RATE_WINDUP = [500., 80., 15.]     #deg/s windup rate limit
ANGLE_RATE_UNWIND = [500., 350., 40.]  #deg/s unwind rate limit

def calc_steering_torque_hold(angle, vEgo, steerActuatorParams):
  angle_sign = 1 if angle > 0 else -1
  speed_dep_linear_curve = SteerActuatorParams.STEER_TORQUE_OFFSET + angle_sign * max(abs(angle), steerActuatorParams.STEER_LINEAR_REGION) *vEgo ** 2 * steerActuatorParams.CENTERING_COEFF
  k = min(abs(angle), steerActuatorParams.STEER_LINEAR_REGION) / steerActuatorParams.STEER_LINEAR_REGION
  return -((1-k)* steerActuatorParams.ZERO_ANGLE_HOLD_TQ * angle_sign + k*speed_dep_linear_curve) #interpolate between zero hold torque and linear region starting point at a given vehicle speed

def accel_hysteresis(accel, accel_steady, enabled):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if not enabled:
    # send 0 when disabled, otherwise acc faults
    accel_steady = 0.
  elif accel > accel_steady + CarControllerParams.ACCEL_HYST_GAP:
    accel_steady = accel - CarControllerParams.ACCEL_HYST_GAP
  elif accel < accel_steady - CarControllerParams.ACCEL_HYST_GAP:
    accel_steady = accel + CarControllerParams.ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady


def coast_accel(speed):  # given a speed, output coasting acceleration
  points = [[0.0, 0.03], [.166, .424], [.335, .568],
            [.731, .440], [1.886, 0.262], [2.809, -0.207],
            [3.443, -0.249], [MIN_ACC_SPEED, -0.145]]
  return interp(speed, *zip(*points))


def compute_gb_pedal(accel, speed):
  _a3, _a4, _a5, _offset, _e1, _e2, _e3, _e4, _e5, _e6, _e7, _e8 = [-0.07264304340456754, -0.007522016704006004, 0.16234124452228196, 0.0029096574419830296, 1.1674372321165579e-05, -0.008010070095545522, -5.834025253616562e-05, 0.04722441060805912, 0.001887454016549489, -0.0014370672920621269, -0.007577594283906699, 0.01943515032956308]
  speed_part = (_e5 * accel + _e6) * speed ** 2 + (_e7 * accel + _e8) * speed
  accel_part = ((_e1 * speed + _e2) * accel ** 5 + (_e3 * speed + _e4) * accel ** 4 + _a3 * accel ** 3 + _a4 * accel ** 2 + _a5 * accel)
  return speed_part + accel_part + _offset


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.accel_steady = 0.
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.standstill_hack = opParams().get('standstill_hack')

    self.steer_rate_limited = False
    
    # StepperServo variables, redundant safety check with the board
    self.last_target_angle_lim = 0
    self.angle_control = False
    self.steer_angle_enabled = False
    self.last_fault_frame = -200
    self.steer_rate_limited = False 
    
    self.fake_ecus = set()
    if CP.enableCamera:
      self.fake_ecus.add(Ecu.fwdCamera)
    if CP.enableDsu:
      self.fake_ecus.add(Ecu.dsu)

    self.packer = CANPacker(dbc_name)

    self.lead_v = 100
    self.lead_a = 0
    self.lead_d = 250
    self.sm = messaging.SubMaster(['radarState', 'controlsState'])
    #self.sm = messaging.SubMaster(['radarState'])
    
    self.LCS = ""
    #self.cm = messaging.Submaster(['controlsState'])


  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):

    # *** compute control surfaces ***
    self.sm.update(0)
    if self.sm.updated['radarState']:
      self.lead_v = self.sm['radarState'].leadOne.vRel
      self.lead_a = self.sm['radarState'].leadOne.aRel
      self.lead_d = self.sm['radarState'].leadOne.dRel
    if self.sm.updated['controlsState']:
      self.LCS = self.sm['controlsState'].longControlState
      if frame % 500 == 0:    # *** 5 sec interval? ***
        print(self.LCS)

    # gas and brake
#    apply_gas = 0.
#    apply_accel = actuators.gas - actuators.brake

#    if CS.CP.enableGasInterceptor and enabled and CS.out.vEgo < MIN_ACC_SPEED:
#      apply_gas = clip(actuators.gas, 0., 1.)
#      # converts desired acceleration to gas percentage for pedal
#      # +0.06 offset to reduce ABS pump usage when applying very small gas
#      if apply_accel * CarControllerParams.ACCEL_SCALE > coast_accel(CS.out.vEgo):
#        apply_gas = clip(compute_gb_pedal(apply_accel * CarControllerParams.ACCEL_SCALE, CS.out.vEgo), 0., 1.)
#      apply_accel += 0.06

    apply_gas = clip(actuators.gas, 0., 1.)

    if CS.CP.enableGasInterceptor:
      # send only negative accel if interceptor is detected. otherwise, send the regular value
      # +0.06 offset to reduce ABS pump usage when OP is engaged
      #apply_accel = 0.06 - actuators.brake    # Original
      if lead:
        apply_accel = 0.06 - actuators.brake
        #apply_accel = 0.06
      else:
        apply_accel = 0.06
      # End new
    else:
      apply_accel = actuators.gas - actuators.brake

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady, enabled)
    apply_accel = clip(apply_accel * CarControllerParams.ACCEL_SCALE, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    # steer torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    # This original gave jitters to StepperServo when motor torque was given
    #apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, CarControllerParams)
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, 0, CarControllerParams)
    steer_tq = apply_steer / 22.4
        steer_tq = steer_tq * (-1)    # Switch StepperServo rotation
    self.steer_rate_limited = new_steer != apply_steer

    # Cut steering while we're in a known fault state (2s)
    if not enabled or abs(CS.out.steeringRateDeg) > 100:
    #if not enabled or CS.steer_state in [9, 25] or CS.out.epsDisabled==1 or abs(CS.out.steeringRateDeg) > 100:    #Original statement
      apply_steer = 0
      steer_tq = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    #if not enabled and CS.pcm_acc_status:    # Original
    if not enabled:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = 1

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR and not self.standstill_hack:
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_steer = apply_steer
    self.last_accel = apply_accel
    self.last_standstill = CS.out.standstill

    can_sends = []
    
    if (frame%2==0):
      can_sends.append(create_lead_command(self.packer, self.lead_v, self.lead_a, self.lead_d))

      
      
# ############################# New Steer Logik ####################################

#     # Cut steering for 2s after fault
#     apply_hold_torque = 0
#     if not control.enabled or (frame - self.last_fault_frame < 200):
#       apply_steer_req = 0
#     else:
#       apply_steer_req = 1

#     # steer angle
#     if control.enabled:
#       angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX)
#       target_angle_lim = clip(actuators.steerAngle, -angle_lim, angle_lim)
      
#       # windup slower #todo implement real (speed) rate limiter
#       if (self.last_target_angle_lim * target_angle_lim) > 0. and abs(target_angle_lim) > abs(self.last_target_angle_lim): #todo revise last_angle
#         angle_delta_lim = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_WINDUP) 
#       else:
#         angle_delta_lim = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_UNWIND)
#       angle_max_rate = angle_delta_lim / SAMPLING_FREQ
      
#       # steer angle - don't allow too large delta
#       MAX_SEC_BEHIND = 1 #seconds behind target. Target deltas behind more than 1s will be rejected by bmw_safety
#       target_angle_lim = clip(target_angle_lim, self.last_target_angle_lim - angle_delta_lim*MAX_SEC_BEHIND, self.last_target_angle_lim + angle_delta_lim*MAX_SEC_BEHIND)
      
#       target_angle_delta = CS.out.steeringAngle - target_angle_lim 
#       angle_desired_rate = clip(target_angle_delta, -angle_max_rate, angle_max_rate) #apply max allowed rate such that the target is not overshot within a sample
      
#       self.steer_rate_limited = target_angle_delta != angle_desired_rate #desired rate only drives stepper (inertial) holding torque in this iteration. Rate is limited independently in Trinamic controller
      
#       # steer torque
#       I_steering = 0.05 #estimated moment of inertia (inertia of a ring = I=mR^2 = 2kg * .15^2 = 0.045kgm2)
#       inertia_tq = I_steering * ((angle_desired_rate * SAMPLING_FREQ - CS.out.steeringRate ) * SAMPLING_FREQ) * CV.DEG_TO_RAD  #kg*m^2 * rad/s^2 = N*m (torque)
      
#       # add friciton compensation feed-forward
#       steer_tq = calc_steering_torque_hold(CS.out.vEgo, target_angle_lim, SteerActuatorParams) + inertia_tq

#       # explicitly clip torque before sending on CAN
#       steer_tq = clip(steer_tq, -SteerActuatorParams.MAX_STEERING_TQ, SteerActuatorParams.MAX_STEERING_TQ)
      
#       can_sends.append(create_new_steer_command(self.packer, int(True), target_angle_delta, steer_tq, frame))
#       # *** control msgs ***
#       if (frame % 10) == 0: #slow print
#         print("SteerAngleErr {0} Inertia  {1} Brake {2}, SpeedDiff {3}".format(control.actuators.steerAngle - CS.out.steeringAngle,
#                                                                  inertia_tq,
#                                                                  control.actuators.brake, speed_diff_req))
#     else:
#       target_angle_lim = CS.out.steeringAngle
#       can_sends.append(create_new_steer_command(self.packer, int(False), 0., 0., frame)) 

#     self.last_target_angle_lim = target_angle_lim
    
# ########################################## End of new Steer Logik #################################################
      
      
      
      
    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)
      # *** control msgs ***
    # if (frame % 30) == 0: #slow print
    #   print("apply_steer {0} steer_tq {1}".format(apply_steer, steer_tq))
    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    if Ecu.fwdCamera in self.fake_ecus:
      # Original steer_command
      can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))
      # StepperServoCan steer_command
      can_sends.append(create_new_steer_command(self.packer, apply_steer_req, 0, steer_tq, frame)) 
      if frame % 2 == 0 and CS.CP.carFingerprint in TSS2_CAR:
        can_sends.append(create_lta_steer_command(self.packer, 0, 0, frame // 2))

      # LTA mode. Set ret.steerControlType = car.CarParams.SteerControlType.angle and whitelist 0x191 in the panda
      # if frame % 2 == 0:
      #   can_sends.append(create_steer_command(self.packer, 0, 0, frame // 2))
      #   can_sends.append(create_lta_steer_command(self.packer, actuators.steeringAngleDeg, apply_steer_req, frame // 2))

    # we can spam can to cancel the system even if we are using lat only control
    if (frame % 3 == 0 and CS.CP.openpilotLongitudinalControl) or (pcm_cancel_cmd and Ecu.fwdCamera in self.fake_ecus):
      lead = lead or CS.out.vEgo < 12.    # at low speed we always assume the lead is present do ACC can be engaged

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and CS.CP.carFingerprint == CAR.LEXUS_IS:
        can_sends.append(create_acc_cancel_command(self.packer))
      elif CS.CP.openpilotLongitudinalControl:
        can_sends.append(create_accel_command(self.packer, apply_accel, pcm_cancel_cmd, self.standstill_req, lead))
      else:
        #can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead))   # Original value
        can_sends.append(create_accel_command(self.packer, 500, pcm_cancel_cmd, False, lead))

    if (frame % 2 == 0) and (CS.CP.enableGasInterceptor):
      # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_command(self.packer, apply_gas, frame//2))

    # ui mesg is at 100Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    fcw_alert = hud_alert == VisualAlert.fcw
    steer_alert = hud_alert == VisualAlert.steerRequired

    send_ui = False
    if ((fcw_alert or steer_alert) and not self.alert_active) or \
       (not (fcw_alert or steer_alert) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    elif pcm_cancel_cmd:
      # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
      send_ui = True

    if (frame % 100 == 0 or send_ui) and Ecu.fwdCamera in self.fake_ecus:
      can_sends.append(create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, left_line, right_line, left_lane_depart, right_lane_depart))

    if frame % 100 == 0 and Ecu.dsu in self.fake_ecus:
      can_sends.append(create_fcw_command(self.packer, fcw_alert))

    #*** static msgs ***

    for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
      if frame % fr_step == 0 and ecu in self.fake_ecus and CS.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    return can_sends
