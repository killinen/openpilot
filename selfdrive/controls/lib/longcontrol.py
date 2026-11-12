import math

from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
GAS_INTERCEPTOR_OVERSPEED_MARGIN = 0.25  # m/s
GAS_INTERCEPTOR_TARGET_ACCEL_MARGIN = 0.05  # m/s

LongCtrlState = car.CarControl.Actuators.LongControlState

# i30 pedal map (old API): integral only trims model error, so it is bounded and may be
# negative; source changes between map and legacy conversion are made bumpless.
PEDAL_MAP_INTEGRATOR_LIMIT = 0.5  # m/s^2
PEDAL_TRANSFER_TIME = 1.0  # s of upward pedal slew limit after a map/legacy switch
PEDAL_TRANSFER_RATE = 0.4  # pedal fraction per second
PEDAL_MAP_REENTRY_TIME = 0.5  # s the map must be continuously available before leaving the legacy fallback


def load_pedal_map(CP):
  """Fitted i30 pedal map and the legacy accel-per-full-pedal, or (None, None)."""
  if CP.carName != "i30" or not CP.enableGasInterceptor:
    return None, None
  from openpilot.selfdrive.car.i30.values import I30_PEDAL_MAP_ENABLED, I30_LEGACY_PEDAL_ACCEL
  if not I30_PEDAL_MAP_ENABLED:
    return None, None
  from openpilot.selfdrive.car.i30.longitudinal_map import I30LongitudinalMap
  return I30LongitudinalMap(), I30_LEGACY_PEDAL_ACCEL


def gas_interceptor_integrator_limits(v_ego, v_pid, v_target_1sec, p_term, feedforward):
  integrator_min = 0.0
  integrator_max = None

  above_target = v_ego > v_pid + GAS_INTERCEPTOR_OVERSPEED_MARGIN
  target_not_accelerating = v_target_1sec <= v_pid + GAS_INTERCEPTOR_TARGET_ACCEL_MARGIN
  if above_target and target_not_accelerating:
    # Keep the integral only as high as can be cancelled by P/FF. This stops
    # positive throttle while overspeed without dumping useful integral during
    # a large cruise target drop where P is already strongly negative.
    integrator_max = max(0.0, -p_term - feedforward)

  return integrator_min, integrator_max


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill, frogpilot_toggles):
  # Ignore cruise standstill if car has a gas interceptor
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > frogpilot_toggles.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

def long_control_state_trans_old_long(CP, active, long_control_state, v_ego, v_target,
                                      v_target_1sec, brake_pressed, cruise_standstill, frogpilot_toggles):
  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < frogpilot_toggles.vEgoStopping and
                  v_target_1sec < frogpilot_toggles.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < frogpilot_toggles.vEgoStopping and
                  (brake_pressed or cruise_standstill))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > frogpilot_toggles.vEgoStarting and
                        accelerating and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > frogpilot_toggles.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0

    self.pedal_map, self.legacy_pedal_accel = load_pedal_map(CP)
    self.pedal_command = 0.0
    self.pedal_from_map = False
    self.pedal_saturation = 0  # -1 at zero pedal (coasting), +1 at the usable pedal limit
    self.pedal_transfer_timer = 0.0
    self.pedal_was_active = False
    self.pedal_map_available_time = 0.0

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits, frogpilot_toggles):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill, frogpilot_toggles)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > frogpilot_toggles.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= frogpilot_toggles.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = (a_target if frogpilot_toggles.human_acceleration else frogpilot_toggles.startAccel)
      self.reset()

    else:  # LongCtrlState.pid
      error = a_target - CS.aEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def reset_old_long(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid
    self.pedal_saturation = 0

  def update_old_long(self, active, CS, long_plan, accel_limits, t_since_plan, frogpilot_toggles, pitch=float('nan')):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # The old API follows trajectory arrays, so scalar aTarget clipping alone
    # does not enforce the planner's throttle veto on the pedal-only i30.
    throttle_blocked = self.CP.carName == "i30" and self.CP.enableGasInterceptor and not long_plan.allowThrottle
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target_now = interp(t_since_plan, CONTROL_N_T_IDX, long_plan.accels)

      v_target = interp(frogpilot_toggles.longitudinalActuatorDelay + t_since_plan, CONTROL_N_T_IDX, speeds)
      a_target = 2 * (v_target - v_target_now) / frogpilot_toggles.longitudinalActuatorDelay - a_target_now

      v_target_1sec = interp(frogpilot_toggles.longitudinalActuatorDelay + t_since_plan + 1.0, CONTROL_N_T_IDX, speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans_old_long(self.CP, active, self.long_control_state, CS.vEgo,
                                                                v_target, v_target_1sec, CS.brakePressed,
                                                                CS.cruiseState.standstill, frogpilot_toggles)

    if self.long_control_state == LongCtrlState.off:
      self.reset_old_long(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      if output_accel > frogpilot_toggles.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= frogpilot_toggles.stoppingDecelRate * DT_CTRL
      self.reset_old_long(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = frogpilot_toggles.startAccel
      self.reset_old_long(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot or throttle_blocked

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)

      integrator_min = None
      integrator_max = None
      if self.CP.enableGasInterceptor:
        # Gas interceptor cars cannot realize negative acceleration commands;
        # negative accel just becomes zero gas. Prevent negative integral windup
        # during long coast-downs so throttle can resume as soon as speed reaches
        # the new setpoint. Also cap positive integral while overspeed so stored
        # ramp-up integral cannot keep requesting throttle above the capped target.
        p_term = interp(CS.vEgo, self.CP.longitudinalTuning.kpBP, self.CP.longitudinalTuning.kpV) * error_deadzone
        integrator_min, integrator_max = gas_interceptor_integrator_limits(CS.vEgo, self.v_pid, v_target_1sec, p_term, a_target)
        if self.pedal_from_map:
          # The map supplies road load and grade, so the integral only trims model error:
          # bounded both ways, and frozen while the pedal is saturated in the error's direction.
          integrator_min = -PEDAL_MAP_INTEGRATOR_LIMIT
          integrator_max = PEDAL_MAP_INTEGRATOR_LIMIT if integrator_max is None else clip(integrator_max, -PEDAL_MAP_INTEGRATOR_LIMIT,
                                                                                           PEDAL_MAP_INTEGRATOR_LIMIT)
          freeze_integrator |= self.pedal_saturation * error_deadzone > 0

      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator,
                                     integrator_min=integrator_min,
                                     integrator_max=integrator_max)

    # Apply to every state, including starting. Preserve negative lead demand;
    # the interceptor converts nonpositive commands to exactly zero pedal.
    if throttle_blocked:
      output_accel = min(output_accel, 0.0)
    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    if self.pedal_map is not None:
      max_accel = getattr(frogpilot_toggles, "max_desired_acceleration", math.inf)
      self.update_pedal_command(active, CS, min(self.last_output_accel, max_accel), throttle_blocked, pitch, max_accel)

    return self.last_output_accel

  def update_pedal_command(self, active, CS, accel, throttle_blocked, pitch, max_accel=math.inf):
    """Interceptor command (0..1) for the old-long i30 path, published via actuators.gas.

    Uses the fitted map when the operating point is covered, otherwise the legacy linear
    conversion. The throttle veto and inactive state always give exactly zero.
    """
    v_ego, rpm = CS.vEgo, getattr(CS, "engineRpm", math.nan)
    legacy = clip(accel / self.legacy_pedal_accel, 0.0, 1.0)
    mapped = self.pedal_map.command(accel, v_ego, rpm, pitch) if active else None
    from_map = mapped is not None

    if not active:
      self.pedal_command, self.pedal_from_map, self.pedal_saturation, self.pedal_transfer_timer = 0.0, False, 0, 0.0
      self.pedal_was_active = False
      self.pedal_map_available_time = 0.0
      return

    # Hysteresis: once on the legacy fallback, return to the map only after it has been
    # available continuously, so a coverage boundary cannot make the source chatter.
    self.pedal_map_available_time = self.pedal_map_available_time + DT_CTRL if from_map else 0.0
    if from_map and self.pedal_was_active and not self.pedal_from_map and self.pedal_map_available_time < PEDAL_MAP_REENTRY_TIME:
      from_map = False

    previous = self.pedal_command
    # Transfer only between two active cycles; on (re)engagement the integral starts from reset.
    if self.pedal_was_active and from_map != self.pedal_from_map and self.long_control_state == LongCtrlState.pid:
      # Bumpless transfer: choose the integral so the new conversion reproduces the last pedal.
      ff_p = self.pid.p + self.pid.f
      if from_map:
        a_prev = self.pedal_map.accel(previous, v_ego, rpm, pitch)
        if a_prev is not None:
          # If the last pedal is at or beyond what the map can command, a_prev is only a lower
          # bound (the map caps there): never push the integral negative on that basis.
          limit = self.pedal_map.command(1e3, v_ego, rpm, pitch)
          at_limit = limit is not None and self.pedal_map.last_limited and previous >= limit - 1e-6
          i_min = 0.0 if at_limit else -PEDAL_MAP_INTEGRATOR_LIMIT
          self.pid.i = clip(a_prev - ff_p, i_min, PEDAL_MAP_INTEGRATOR_LIMIT)
      else:
        self.pid.i = max(previous * self.legacy_pedal_accel - ff_p, 0.0)
      # Re-evaluate this cycle with the transferred integral (vetoes still force zero below).
      accel = clip(ff_p + self.pid.i, self.pid.neg_limit, self.pid.pos_limit)
      self.last_output_accel = min(accel, 0.0) if throttle_blocked else accel
      accel = min(accel, max_accel)
      legacy = clip(accel / self.legacy_pedal_accel, 0.0, 1.0)
      if from_map:
        mapped = self.pedal_map.command(accel, v_ego, rpm, pitch)
        from_map = mapped is not None
      self.pedal_transfer_timer = PEDAL_TRANSFER_TIME
    self.pedal_from_map = from_map

    pedal = mapped if from_map else legacy
    if self.pedal_transfer_timer > 0.0:
      # Only increases are slew limited; releasing the pedal is never delayed.
      self.pedal_transfer_timer = max(self.pedal_transfer_timer - DT_CTRL, 0.0)
      pedal = min(pedal, previous + PEDAL_TRANSFER_RATE * DT_CTRL)
    if throttle_blocked:
      pedal = 0.0
    self.pedal_was_active = True

    if from_map:
      self.pedal_saturation = -1 if pedal <= 0.0 else (1 if self.pedal_map.last_limited else 0)
    else:
      self.pedal_saturation = 0
    self.pedal_command = float(pedal)
