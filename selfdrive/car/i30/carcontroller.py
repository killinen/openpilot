from common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.i30 import i30can
from selfdrive.car.i30.values import CarControllerParams, SteerLimitParams

SAMPLING_FREQ = 100 #Hz

# Steer angle limits
ANGLE_MAX_BP = [5., 15., 30]  #m/s (8, 54, 108 km/h)
ANGLE_MAX = [200., 30., 15.] #deg
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


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    self.apply_steer_last = 0
    self.steer_rate_limited = False
    self.accel = 0
    self.gas = 0

    # StepperServo variables, redundant safety check with the board
    self.last_target_angle_lim = 0
    self.last_fault_frame = -200
    self.target_angle_delta = 0
    self.last_steer_tq = 0

  def update(self, CC, CS):
    actuators = CC.actuators

    steer = actuators.steer
    new_steer = int(round(steer * self.params.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
    self.steer_rate_limited = new_steer != apply_steer

    if not CC.latActive:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    # latActive is when OP latControl is ON
    if not CC.latActive:
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    # Cut steering for 2s after fault
    steer_tq = 0
    angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX)
    target_angle_lim = clip(actuators.steeringAngleDeg, -angle_lim, angle_lim)
    if CC.enabled:
      # windup slower
      if (self.last_target_angle_lim * target_angle_lim) > 0. and abs(target_angle_lim) > abs(self.last_target_angle_lim):
        angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_WINDUP)
      else:
        angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_UNWIND)
      target_angle_lim = clip(target_angle_lim, self.last_target_angle_lim - angle_rate_max, self.last_target_angle_lim + angle_rate_max)
      self.target_angle_delta = target_angle_lim - CS.out.steeringAngleDeg
      angle_step_max = angle_rate_max / SAMPLING_FREQ  #max angle step per single sample
      angle_step = clip(self.target_angle_delta, -angle_step_max, angle_step_max) #apply angle step
      self.steer_rate_limited = self.target_angle_delta != angle_step

      raw_steer_tq = actuators.steer * SteerLimitParams.STEER_MAX

      # Filter the output to reduce actuator jitter
      alpha = 0.35  # Lower = smoother but more lag

      # First order low pass filter
      raw_steer_tq = lowpass_filter(raw_steer_tq, self.last_steer_tq, alpha)

      # Apply steering torque derivative limits
      steer_tq = apply_ssc_steer_torque_limits(raw_steer_tq, self.last_steer_tq, SteerLimitParams)

      self.last_steer_tq = steer_tq

    can_sends = []

    # Send SSC steering command on platforms that expect the standalone steering message
    can_sends.append(i30can.create_steer_command(self.packer, apply_steer_req, self.target_angle_delta, steer_tq, self.frame))

    # Longitudinal + gas interceptor (bus 1)
    self.accel = 0.0
    if self.CP.openpilotLongitudinalControl:
      self.accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX) if CC.longActive else 0.0

    if self.CP.openpilotLongitudinalControl and self.CP.enableGasInterceptor:
      if CC.longActive:
        pedal_command = interp(self.accel, [0.0, 1.6], [0.0, 0.7])
        interceptor_gas_cmd = clip(pedal_command, 0.0, 0.7)
      else:
        interceptor_gas_cmd = 0.0

      # Send exactly zero when disabled; this prevents unexpected pedal range rescaling in the interceptor.
      if self.frame % 2 == 0:
        can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))
      self.gas = interceptor_gas_cmd
    else:
      self.gas = 0.0

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
