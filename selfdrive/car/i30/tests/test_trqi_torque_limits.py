"""TRQI headroom must not rescale ordinary torque or relax driver takeover."""
from types import SimpleNamespace

import pytest
from cereal import car
from panda import Panda

from openpilot.selfdrive.car.i30.interface import CarInterface
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.car.i30.tests.test_steering_mode import controller_modules, new_controller


@pytest.mark.parametrize('toggle,trqi', [(b'0', False), (b'1', True)])
def test_safety_param_tracks_selected_backend(tmp_path, monkeypatch, toggle, trqi):
  controller_modules(tmp_path, monkeypatch, toggle)
  monkeypatch.setattr(CarInterfaceBase, 'configure_torque_tune', lambda *args: None)
  cp = car.CarParams.new_message()
  CarInterface._get_params(cp, None, None, None, False, False, None)
  assert cp.safetyConfigs[0].safetyParam == Panda.FLAG_I30_LONGITUDINAL | (Panda.FLAG_I30_TRQI if trqi else 0)


def state(output_torque=0.0, pressed=False, snr_diff_mv=0.0, requested=False, pending=False, left_blinker=False, right_blinker=False, v_ego=15.0):
  out = car.CarState.new_message(steeringTorque=0.0, steeringPressed=pressed, leftBlinker=left_blinker, rightBlinker=right_blinker, vEgo=v_ego)
  return SimpleNamespace(out=out, steering_torque_out=output_torque,
                         trqi_snr_diff_mv=snr_diff_mv, trqi_snr_passthrough_requested=requested, trqi_snr_passthrough_pending=pending)


def command(steer, lane_change_left=False, lane_change_right=False):
  cc = car.CarControl.new_message(latActive=True, leftBlinker=lane_change_left, rightBlinker=lane_change_right)
  cc.actuators.steer = steer
  return cc.as_reader()


def run(controller, steer, frames, **kw):
  cmd_kw = {k: kw.pop(k) for k in ('lane_change_left', 'lane_change_right') if k in kw}
  return [demand(controller.update(command(steer, **cmd_kw), state(**kw), 0, SimpleNamespace())[1]) for _ in range(frames)]


def demand(messages):
  payload = next(m[2] for m in messages if m[0] == 0x232)
  raw = int.from_bytes(payload[:2], 'little') & 0xFFF
  return raw - 0x1000 if raw & 0x800 else raw


@pytest.mark.parametrize('sign', [-1, 1])
def test_reaches_twelve_nm_with_nominal_engage_ramp_then_catchup(tmp_path, monkeypatch, sign):
  _, values, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller, _ = new_controller(module)
  cc = command(sign*1.2)
  outputs = []
  for _ in range(205):
    applied, messages = controller.update(cc, state(), 0, SimpleNamespace())
    outputs.append(demand(messages))
  steps = [abs(b-a) for a, b in zip([0, *outputs], outputs, strict=False)]
  limits = values.TrqiSteerLimitParams
  # The engage ramp keeps the nominal slew; catch-up starts only after it.
  assert outputs[0] == sign*6
  assert max(steps[:limits.CATCHUP_ENGAGE_FRAMES - 1]) == 6
  assert max(steps) == 15
  assert outputs[-1] == sign*1200
  assert applied.steer == pytest.approx(sign*1.2)
  applied, messages = controller.update(command(sign*1.5), state(), 0, SimpleNamespace())
  assert demand(messages) == sign*1200
  assert values.TrqiSteerLimitParams.STEER_MAX == 1000


def settled(module, tq, frames=200):
  controller, _ = new_controller(module)
  for _ in range(frames):
    controller.update(command(tq / 1000), state(), 0, SimpleNamespace())
  assert controller.last_trqi_tq == pytest.approx(tq)
  return controller


@pytest.mark.parametrize('sign', [-1, 1])
def test_catchup_unwinds_curve_exit_and_returns_to_nominal(tmp_path, monkeypatch, sign):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller = settled(module, sign*1000)
  outputs = [demand(controller.update(command(-sign*1.0), state(), 0, SimpleNamespace())[1]) for _ in range(200)]
  steps = [b-a for a, b in zip([sign*1000, *outputs], outputs, strict=False)]
  assert steps[0] == -sign*15
  # Once within the gap the nominal 6 Ncm/frame finishes the move.
  gap = [abs(-sign*1000 - o) for o in outputs]
  assert all(abs(st) == 6 for st, g in zip(steps[1:], gap, strict=False) if 6 < g <= 300)
  assert outputs[-1] == -sign*1000


def test_small_gap_keeps_nominal_slew(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller = settled(module, 200)
  _, messages = controller.update(command(.5), state(), 0, SimpleNamespace())
  assert demand(messages) == 206  # gap exactly 300 does not boost
  _, messages = controller.update(command(.51), state(), 0, SimpleNamespace())
  assert demand(messages) == 221  # 304 > 300 boosts


DRIVER_FLAGS = [dict(pressed=True), dict(requested=True), dict(pending=True)]


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('driver', DRIVER_FLAGS)
def test_driver_flags_block_catchup_both_ways_with_holdoff(tmp_path, monkeypatch, driver, sign):
  # Passthrough active (steeringPressed), requested or pending: nominal slew for unwind and
  # windup, at any speed, until the hold-off has run out.
  _, values, module = controller_modules(tmp_path, monkeypatch, b'1')
  holdoff = values.TrqiSteerLimitParams.CATCHUP_HOLDOFF_FRAMES
  for v_ego in (8.0, 30.0):
    controller = settled(module, sign*1000)
    assert run(controller, -sign*1.0, 1, v_ego=v_ego, **driver) == [sign*994]
    outputs = run(controller, -sign*1.0, holdoff, v_ego=v_ego)
    # Nominal slew for the driver frame plus holdoff - 1 frames, then catch-up.
    assert outputs[holdoff - 2] == sign*(994 - 6*(holdoff - 1))
    assert outputs[holdoff - 1] == outputs[holdoff - 2] - sign*15
  controller = settled(module, 0)
  assert run(controller, sign*1.0, 1, v_ego=8.0, **driver) == [sign*6]


@pytest.mark.parametrize('sign', [-1, 1])
def test_analog_only_driver_estimate_keeps_low_speed_unwind(tmp_path, monkeypatch, sign):
  # Tight-turn exit: the SNR estimate reads ~0.5 Nm at 10-12 Nm EPS output with no flag set.
  _, values, module = controller_modules(tmp_path, monkeypatch, b'1')
  limit = values.TrqiSteerLimitParams.CATCHUP_ANALOG_UNWIND_MAX_SPEED
  controller = settled(module, sign*1000)
  outputs = run(controller, -sign*1.0, 90, v_ego=limit - 1.0, snr_diff_mv=200.0)
  steps = [b-a for a, b in zip([sign*1000, *outputs], outputs, strict=False)]
  assert steps[0] == -sign*15
  # Crossing zero into the opposite windup stays nominal while the analog hold-off lasts.
  crossed = next(i for i, o in enumerate(outputs) if o*sign < 0)
  assert abs(outputs[crossed]) <= 6
  assert all(abs(st) <= 6 for st in steps[crossed + 1:])
  # Above the speed limit the analog estimate blocks unwind too.
  controller = settled(module, sign*1000)
  assert run(controller, -sign*1.0, 1, v_ego=limit + 1.0, snr_diff_mv=200.0) == [sign*994]


@pytest.mark.parametrize('mv,step', [(169.0, 15), (170.0, 6)])
def test_analog_threshold_blocks_windup(tmp_path, monkeypatch, mv, step):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller = settled(module, 0)
  assert run(controller, 1.0, 1, v_ego=12.0, snr_diff_mv=mv) == [step]


LANE_SIGNALS = [dict(left_blinker=True), dict(right_blinker=True), dict(lane_change_left=True), dict(lane_change_right=True)]


@pytest.mark.parametrize('signal', LANE_SIGNALS)
def test_lane_change_blocks_catchup_with_holdoff(tmp_path, monkeypatch, signal):
  # Physical blinker, or the model lane change (CC blinkers) outlasting it.
  _, values, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller = settled(module, 1000)
  on = run(controller, -1.0, 100, **signal)
  assert on[-1] == 1000 - 6*100
  holdoff = values.TrqiSteerLimitParams.CATCHUP_LANE_CHANGE_HOLDOFF_FRAMES
  off = run(controller, -1.0, holdoff)
  # Nominal for holdoff - 1 frames after the signal clears, then catch-up resumes.
  assert off[holdoff - 2] == on[-1] - 6*(holdoff - 1)
  assert off[holdoff - 1] == off[holdoff - 2] - 15


@pytest.mark.parametrize('signal', LANE_SIGNALS)
def test_lane_change_gate_needs_speed_above_threshold(tmp_path, monkeypatch, signal):
  # Junction or roundabout below 50 km/h with the blinker on keeps catch-up.
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller = settled(module, 1000)
  assert run(controller, -1.0, 1, v_ego=12.0, **signal) == [985]


def test_lane_change_gate_latches_for_the_episode(tmp_path, monkeypatch):
  _, values, module = controller_modules(tmp_path, monkeypatch, b'1')
  limit = values.TrqiSteerLimitParams.CATCHUP_LANE_CHANGE_MIN_SPEED
  # Signal starts above the threshold, speed then drops below it: stays gated.
  controller = settled(module, 1000)
  run(controller, 1.0, 10, v_ego=limit + 1.0, right_blinker=True)
  assert run(controller, -1.0, 1, v_ego=limit - 3.0, right_blinker=True) == [994]
  # Below the threshold first, then accelerating above it: gated from then on.
  controller = settled(module, 1000)
  assert run(controller, -1.0, 1, v_ego=limit - 3.0, right_blinker=True) == [985]
  assert run(controller, -1.0, 1, v_ego=limit + 1.0, right_blinker=True) == [979]
  assert run(controller, -1.0, 1, v_ego=limit - 3.0) == [973]
  # The latch clears once the hold-off has run out.
  controller = settled(module, 1000)
  run(controller, 1.0, 10, v_ego=limit + 1.0, right_blinker=True)
  run(controller, 1.0, values.TrqiSteerLimitParams.CATCHUP_LANE_CHANGE_HOLDOFF_FRAMES, v_ego=limit - 3.0)
  assert run(controller, -1.0, 1, v_ego=limit - 3.0) == [985]


@pytest.mark.parametrize('v_ego,windup_step', [(15.0, 15), (19.44, 15), (22.22, 10.5), (25.0, 6), (33.0, 6)])
def test_windup_catchup_fades_with_speed(tmp_path, monkeypatch, v_ego, windup_step):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller, _ = new_controller(module)
  run(controller, 0.0, 200, v_ego=v_ego)
  # Curve entry: a big request from zero grows at the speed-faded catch-up rate.
  assert abs(run(controller, 1.0, 1, v_ego=v_ego)[0] - windup_step) <= 0.5


@pytest.mark.parametrize('sign', [-1, 1])
def test_unwind_catchup_kept_at_highway_speed(tmp_path, monkeypatch, sign):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller, _ = new_controller(module)
  run(controller, sign*1.0, 200, v_ego=30.0)
  assert controller.last_trqi_tq == pytest.approx(sign*1000)
  outputs = run(controller, -sign*1.0, 120, v_ego=30.0)
  steps = [b-a for a, b in zip([sign*1000, *outputs], outputs, strict=False)]
  # Unwind toward zero at the full catch-up rate, then windup the other way at nominal.
  assert steps[0] == -sign*15
  crossed = next(i for i, o in enumerate(outputs) if o*sign < 0)
  # The crossing step itself: the opposite-side part is bounded by the faded windup rate.
  assert abs(outputs[crossed]) <= 6
  assert all(abs(st) <= 6 for st in steps[crossed + 1:])


def test_catchup_respects_measured_output_guard(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller = settled(module, 600)
  _, messages = controller.update(command(1.2), state(12.1), 0, SimpleNamespace())
  assert demand(messages) == 600
  _, messages = controller.update(command(0.0), state(12.1), 0, SimpleNamespace())
  assert demand(messages) == 585


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('initial', [2, 4, 8])
def test_output_guard_reversal_unwinds_to_zero_and_holds(tmp_path, monkeypatch, sign, initial):
  # Above 12 Nm measured output, a reversing request (nominal slew under the driver gate)
  # must finish the unwind at zero rather than hold residual torque or wind up the other way.
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller = settled(module, sign*initial)
  outputs = run(controller, -sign*1.0, 5, output_torque=12.1, pressed=True)
  assert all(o*sign >= 0 for o in outputs)
  assert outputs[-1] == 0
  # Just below the threshold the reversal proceeds normally.
  controller = settled(module, sign*initial)
  assert run(controller, -sign*1.0, 1, output_torque=11.9, pressed=True) == [sign*(initial - 6)]


def test_overlapping_gates_must_all_clear(tmp_path, monkeypatch):
  # A driver flag that clears before the lane-change hold-off does not re-enable catch-up.
  _, values, module = controller_modules(tmp_path, monkeypatch, b'1')
  holdoff = values.TrqiSteerLimitParams.CATCHUP_HOLDOFF_FRAMES
  lane_holdoff = values.TrqiSteerLimitParams.CATCHUP_LANE_CHANGE_HOLDOFF_FRAMES
  controller = settled(module, 1000)
  run(controller, -1.0, 1, pressed=True, right_blinker=True)
  run(controller, -1.0, 50, right_blinker=True)
  outputs = run(controller, -1.0, max(holdoff, lane_holdoff) + 1)
  steps = [b-a for a, b in zip(outputs, outputs[1:], strict=False)]
  # The driver hold-off has run out inside this window, but the lane-change hold-off (reloaded
  # 50 frames later) has not: nominal until it ends, then catch-up.
  assert all(st == -6 for st in steps[:lane_holdoff - 2])
  assert steps[lane_holdoff - 1] == -15


def test_driver_input_while_disengaged_holds_off_after_reengage(tmp_path, monkeypatch):
  # Hold-offs keep counting while lateral control is off; the engage ramp restarts on re-engage.
  _, values, module = controller_modules(tmp_path, monkeypatch, b'1')
  engage = values.TrqiSteerLimitParams.CATCHUP_ENGAGE_FRAMES
  controller = settled(module, 0)
  off = car.CarControl.new_message(latActive=False).as_reader()
  for _ in range(5):
    controller.update(off, state(pressed=True), 0, SimpleNamespace())
  outputs = run(controller, 1.0, engage + 1)
  steps = [b-a for a, b in zip([0, *outputs], outputs, strict=False)]
  assert all(st == 6 for st in steps[:engage - 1])
  assert steps[engage] == 15


def test_disengage_resets_catchup(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller = settled(module, 1000)
  off = car.CarControl.new_message(latActive=False).as_reader()
  controller.update(off, state(), 0, SimpleNamespace())
  assert controller.last_trqi_tq == 0
  _, messages = controller.update(command(1.0), state(), 0, SimpleNamespace())
  assert demand(messages) == 6


def test_normal_range_torque_scale_is_preserved(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller, _ = new_controller(module)
  for _ in range(60):
    applied, messages = controller.update(command(.3), state(), 0, SimpleNamespace())
  assert demand(messages) == 300  # 0.3 still commands 3 Nm, not 3.6 Nm.
  assert applied.steer == pytest.approx(.3)


def test_measured_output_guard_allows_unwind_at_twelve_nm(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  controller, _ = new_controller(module)
  controller.last_trqi_tq = 1100
  _, messages = controller.update(command(1.2), state(12.1), 0, SimpleNamespace())
  assert demand(messages) == 1100
  _, messages = controller.update(command(.5), state(12.1), 0, SimpleNamespace())
  assert demand(messages) == 1094
  _, messages = controller.update(command(1.2), state(11.9), 0, SimpleNamespace())
  assert demand(messages) == 1100


def test_pid_headroom_extends_without_changing_conversion(tmp_path, monkeypatch):
  _, _, module = controller_modules(tmp_path, monkeypatch, b'1')
  _, cp = new_controller(module)
  cp.steerLimitTimer = .4
  tp = cp.lateralTuning.init('torque')
  tp.latAccelFactor = 2.22
  ci = object.__new__(CarInterface)
  control = LatControlTorque(cp, ci, .01)
  assert control.steer_max == pytest.approx(1.2)
  assert control.pid.pos_limit == pytest.approx(2.664)
  assert control.pid.neg_limit == pytest.approx(-2.664)
  assert ci.torque_from_lateral_accel_linear(.666, tp) == pytest.approx(.3)
  cp.steeringMode = 'legacy_ssc'
  assert CarInterface.get_steer_max(cp) == 1.0
  assert CarInterfaceBase.get_steer_max(cp) == 1.0
