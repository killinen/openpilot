import math

from openpilot.common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits
from openpilot.selfdrive.car.i30 import i30can
from openpilot.selfdrive.car.i30.values import CarControllerParams, SteerLimitParams, TrqiSteerLimitParams, i30_uses_trqi_steering
from openpilot.selfdrive.car.interfaces import CarControllerBase

SAMPLING_FREQ = 100  # Hz

# Steering request low-pass tuning. These are kept separate so the legacy SSC
# path and the TRQI path can be tuned independently without changing behavior in
# the other backend.
TRQI_ALPHA = 1.0
SSC_ALPHA = 0.35
TRQI_OUT_TQ_LIMIT_THRESHOLD = 10.0

# Steer angle limits
ANGLE_MAX_BP = [5., 15., 30]  # m/s (8, 54, 108 km/h)
ANGLE_MAX = [200., 30., 15.]  # deg
ANGLE_RATE_BP = [0., 5., 15.]
ANGLE_RATE_WINDUP = [500., 80., 15.]  # deg/s windup rate limit
ANGLE_RATE_UNWIND = [500., 350., 40.]  # deg/s unwind rate limit

# Simple exponential smoothing
def lowpass_filter(new_val, prev_val, alpha):
  return alpha * new_val + (1 - alpha) * prev_val


# Shared first-order rate limiter used by both steering backends.
def apply_rate_limited_steering_limits(apply_torque, apply_torque_last, LIMITS):
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


def apply_rate_limited_steering_limits_with_flags(apply_torque, apply_torque_last, LIMITS):
  delta_up_limited = False
  delta_down_limited = False
  max_limited = False

  if apply_torque_last > 0:
    lower_down = apply_torque_last - LIMITS.STEER_DELTA_DOWN
    lower_up_floor = -LIMITS.STEER_DELTA_UP
    lower = max(lower_down, lower_up_floor)
    upper = apply_torque_last + LIMITS.STEER_DELTA_UP
    if apply_torque < lower:
      delta_down_limited = lower == lower_down
      delta_up_limited = lower == lower_up_floor
    elif apply_torque > upper:
      delta_up_limited = True
  else:
    lower = apply_torque_last - LIMITS.STEER_DELTA_UP
    upper_down = apply_torque_last + LIMITS.STEER_DELTA_DOWN
    upper_up_cap = LIMITS.STEER_DELTA_UP
    upper = min(upper_down, upper_up_cap)
    if apply_torque < lower:
      delta_up_limited = True
    elif apply_torque > upper:
      delta_down_limited = upper == upper_down
      delta_up_limited = upper == upper_up_cap

  rate_limited = clip(apply_torque, lower, upper)
  max_clipped = clip(rate_limited, -LIMITS.MAX_STEERING_TQ, LIMITS.MAX_STEERING_TQ)
  max_limited = abs(rate_limited - max_clipped) > 1e-6

  return max_clipped, delta_up_limited, delta_down_limited, max_limited


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    self.use_trqi_steering = i30_uses_trqi_steering()

    self.apply_steer_last = 0
    self.steer_rate_limited = False
    self.accel = 0
    self.gas = 0

    # StepperServo variables, redundant safety check with the board
    self.last_target_angle_lim = 0
    self.last_fault_frame = -200
    self.target_angle_delta = 0
    self.last_steer_tq = 0

    # TRQI mode keeps its own command state because the command domain is the
    # sender's synthetic TQ scale, not the old SSC Nm signal.
    self.last_trqi_tq = 0.0
    self.trqi_counter = 0

  def update(self, CC, CS, now_nanos, frogpilot_toggles):
    actuators = CC.actuators
    can_sends = []

    if self.use_trqi_steering:
      # TRQI torque mode only needs the torque-like path. The old standalone angle
      # request does not exist on the TRQI bus, so we translate openpilot steer
      # directly into signed EPS output-torque Ncm demand for 0x232.
      self.target_angle_delta = 0.0
      self.apply_steer_last = 0
      self.last_target_angle_lim = 0.0
      self.last_steer_tq = 0.0

      trqi_limit_flags = 0
      if CC.latActive:
        requested_trqi_tq = actuators.steer * TrqiSteerLimitParams.STEER_MAX

        # Filter the output so the raw Ncm request does not chatter on the board.
        # Higher alpha follows the requested steering faster with less smoothing.
        # Lower alpha smooths more, but it also makes steering response slower.
        alpha = TRQI_ALPHA
        filtered_trqi_tq = lowpass_filter(requested_trqi_tq, self.last_trqi_tq, alpha)
        apply_trqi_tq, delta_up_limited, delta_down_limited, max_limited = apply_rate_limited_steering_limits_with_flags(
          filtered_trqi_tq, self.last_trqi_tq, TrqiSteerLimitParams
        )

        measured_out_tq = abs(getattr(CS, "steering_torque_out", 0.0))
        if measured_out_tq > TRQI_OUT_TQ_LIMIT_THRESHOLD:
          trqi_limit_flags |= i30can.TRQI_LIMIT_FLAG_OUT_TQ_LIMITED

          # Once the measured MDPS output torque is already above the target
          # window, block only further windup. Still allow the controller to
          # unwind toward zero so the measured output torque can fall again.
          if abs(apply_trqi_tq) > abs(self.last_trqi_tq):
            max_limited |= abs(apply_trqi_tq - self.last_trqi_tq) > 1e-6
            apply_trqi_tq = self.last_trqi_tq

        if delta_up_limited:
          trqi_limit_flags |= i30can.TRQI_LIMIT_FLAG_STEER_DELTA_UP
        if delta_down_limited:
          trqi_limit_flags |= i30can.TRQI_LIMIT_FLAG_STEER_DELTA_DOWN
        if max_limited:
          trqi_limit_flags |= i30can.TRQI_LIMIT_FLAG_STEER_MAX

        self.steer_rate_limited = delta_up_limited or delta_down_limited or max_limited or (abs(filtered_trqi_tq - apply_trqi_tq) > 1e-6)
      else:
        apply_trqi_tq = 0.0
        self.steer_rate_limited = False

      self.last_trqi_tq = apply_trqi_tq
      can_sends.append(i30can.create_trqi_torque_command(apply_trqi_tq, CC.latActive, self.trqi_counter, trqi_limit_flags))
      self.trqi_counter = (self.trqi_counter + 1) & 0x0F

      # Report the applied command back in openpilot's native steer sign so the
      # rest of the stack still sees the familiar normalized actuator value.
      applied_steer = 0.0
      if TrqiSteerLimitParams.STEER_MAX != 0:
        applied_steer = apply_trqi_tq / TrqiSteerLimitParams.STEER_MAX
    else:
      self.last_trqi_tq = 0.0

      steer = actuators.steer
      new_steer = int(round(steer * self.params.STEER_MAX))
      apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)
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
      target_angle = actuators.steeringAngleDeg
      if target_angle == 0.0 and actuators.curvature != 0.0:
        # LatControlTorque outputs 0 angle; derive desired angle from curvature for SSC.
        target_angle = math.degrees(self.VM.get_steer_from_curvature(-actuators.curvature, CS.out.vEgo, 0.0))
      target_angle_lim = clip(target_angle, -angle_lim, angle_lim)
      if CC.enabled:
        # windup slower
        if (self.last_target_angle_lim * target_angle_lim) > 0. and abs(target_angle_lim) > abs(self.last_target_angle_lim):
          angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_WINDUP)
        else:
          angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_UNWIND)
        target_angle_lim = clip(target_angle_lim, self.last_target_angle_lim - angle_rate_max, self.last_target_angle_lim + angle_rate_max)
        self.target_angle_delta = target_angle_lim - CS.out.steeringAngleDeg
        angle_step_max = angle_rate_max / SAMPLING_FREQ  # max angle step per single sample
        angle_step = clip(self.target_angle_delta, -angle_step_max, angle_step_max)  # apply angle step
        self.steer_rate_limited = self.target_angle_delta != angle_step

        raw_steer_tq = actuators.steer * SteerLimitParams.STEER_MAX

        # Filter the output to reduce actuator jitter.
        # Higher alpha follows the requested steering faster with less smoothing.
        # Lower alpha smooths more, but it also makes steering response slower.
        alpha = SSC_ALPHA

        # First order low pass filter
        raw_steer_tq = lowpass_filter(raw_steer_tq, self.last_steer_tq, alpha)

        # Apply steering torque derivative limits
        steer_tq = apply_rate_limited_steering_limits(raw_steer_tq, self.last_steer_tq, SteerLimitParams)

        self.last_steer_tq = steer_tq
      else:
        self.last_steer_tq = 0.0
        self.target_angle_delta = 0.0

      # Send SSC steering command on platforms that expect the standalone steering message.
      can_sends.append(i30can.create_steer_command(self.packer, apply_steer_req, self.target_angle_delta, steer_tq, self.frame))
      applied_steer = apply_steer / self.params.STEER_MAX

    # Longitudinal + gas interceptor (bus 1)
    self.accel = 0.0
    if self.CP.openpilotLongitudinalControl:
      self.accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX) if CC.longActive else 0.0

    if self.CP.openpilotLongitudinalControl and self.CP.enableGasInterceptor:
      if CC.longActive:
        pedal_command = interp(self.accel, [0.0, 1.6], [0.0, 1.0])
        interceptor_gas_cmd = clip(pedal_command, 0.0, 1.0)
      else:
        interceptor_gas_cmd = 0.0

      # Send exactly zero when disabled; this prevents unexpected pedal range rescaling in the interceptor.
      if self.frame % 2 == 0:
        can_sends.append(i30can.create_gas_interceptor_command(self.packer, interceptor_gas_cmd, self.frame // 2))
      self.gas = interceptor_gas_cmd
    else:
      self.gas = 0.0

    new_actuators = actuators.as_builder()
    new_actuators.steer = applied_steer
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends
