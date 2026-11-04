from collections import defaultdict
import csv
from types import SimpleNamespace

import pytest

from cereal import car, custom, log
from openpilot.selfdrive.car.toyota.carstate import CarState
from openpilot.selfdrive.car.toyota.hrr_angle_shadow import HrrAngleShadow
from openpilot.selfdrive.car.toyota.values import CAR
from openpilot.tools.tuning.replay_hrr_angle_shadow import export_recorded


def sample(observer, now, counter, **changes):
  args = dict(now=now, oem=19.5, oem_rate=0.0, oem_updated=True, hrr=12.3, hrr_rate=0.0,
              hrr_updated=True, counter=counter, valid=True, wrap=False)
  args.update(changes)
  return observer.update(**args)


def ready(observer):
  for i in range(5):
    result = sample(observer, i * .01, i)
  assert result.mode == 'trustedHrr'


def wrong_sector(observer, start=5):
  """The firmware exit seen in logs: invalidation, then valid HRR one sector high."""
  sample(observer, start * .01, start, valid=False, wrap=True)
  for i in range(start + 1, start + 6):
    result = sample(observer, i * .01, i, hrr=34.8)
  return result


def test_flagged_gap_bridges_on_local_reference_and_tracks_motion():
  ghost = HrrAngleShadow()
  ready(ghost)
  result = sample(ghost, .05, 5, valid=False, wrap=True)
  assert result.mode == 'alignedOem'
  assert result.angle == pytest.approx(12.3)
  moved = sample(ghost, .06, 6, oem=21.0, valid=False, wrap=True)
  assert moved.angle == pytest.approx(13.8)
  assert moved.usable


def test_bridge_covers_longest_logged_gap_then_expires_without_latching():
  ghost = HrrAngleShadow()
  ready(ghost)
  assert sample(ghost, 1.95, 5, valid=False, wrap=True).usable
  result = sample(ghost, 3.0, 6, valid=False, wrap=True)
  assert result.mode == 'unavailable'
  assert result.would_disable
  for i in range(7, 11):
    assert not sample(ghost, 3.0 + i * .01, i).usable
  result = sample(ghost, 3.11, 11)
  assert result.mode == 'trustedHrr'
  assert not result.would_disable


def test_wrong_sector_exit_is_corrected_after_qualifying():
  ghost = HrrAngleShadow()
  ready(ghost)
  sample(ghost, .05, 5, valid=False, wrap=True)
  assert sample(ghost, .06, 6, hrr=34.8).mode == 'alignedOem'
  ghost = HrrAngleShadow()
  ready(ghost)
  result = wrong_sector(ghost)
  assert result.mode == 'sectorCorrectedHrr'
  assert result.reason == 'sectorCorrected'
  assert result.sector_offset == pytest.approx(22.5)
  assert result.angle == pytest.approx(12.3)


def test_sector_stays_fixed_through_oem_excursion():
  ghost = HrrAngleShadow()
  ready(ghost)
  wrong_sector(ghost)
  # OEM reads far high while HRR continuity holds, as in the logged highway events.
  result = sample(ghost, .11, 11, oem=33.0, hrr=34.8)
  assert result.mode == 'sectorCorrectedHrr'
  assert result.angle == pytest.approx(12.3)
  assert result.innovation == pytest.approx(-13.5)


def test_invalidation_reselects_sector():
  ghost = HrrAngleShadow()
  ready(ghost)
  wrong_sector(ghost)
  sample(ghost, .11, 11, hrr=34.8, valid=False, wrap=True)
  for i in range(12, 17):
    result = sample(ghost, i * .01, i)
  assert result.mode == 'trustedHrr'
  assert result.sector_offset == 0.0
  assert result.angle == pytest.approx(12.3)


def test_slow_offset_walk_cannot_poison_later_decisions():
  ghost = HrrAngleShadow()
  ready(ghost)
  # Version 1 followed this walk per sample, then rejected healthy HRR for the rest of the drive.
  for i, hrr in enumerate(range(13, 28)):
    result = sample(ghost, (5 + i) * .01, 5 + i, hrr=float(hrr))
    assert result.mode == 'trustedHrr'
  result = sample(ghost, .20, 20)
  assert result.mode != 'unavailable'
  assert result.usable
  ghost = HrrAngleShadow()
  ready(ghost)
  sample(ghost, .05, 5, valid=False, wrap=True)
  for i in range(6, 11):
    result = sample(ghost, i * .01, i)
  assert result.mode == 'trustedHrr'
  assert result.sector_offset == 0.0


def test_sector_between_margins_is_ambiguous():
  ghost = HrrAngleShadow()
  ready(ghost)
  sample(ghost, .05, 5, valid=False, wrap=True)
  result = sample(ghost, .06, 6, hrr=23.5)
  assert result.reason == 'sectorAmbiguous'
  assert result.mode == 'alignedOem'
  assert result.recovery_samples == 0


def test_valid_jump_without_invalidation_reselects_sector():
  ghost = HrrAngleShadow()
  ready(ghost)
  result = sample(ghost, .05, 5, hrr=34.8)
  assert result.reason == 'inconsistent'
  assert result.mode == 'alignedOem'
  assert result.angle == pytest.approx(12.3)
  for i in range(6, 10):
    result = sample(ghost, i * .01, i, hrr=34.8)
  assert result.mode == 'sectorCorrectedHrr'
  assert result.angle == pytest.approx(12.3)


def test_outside_envelope_never_applies_a_sector():
  ghost = HrrAngleShadow()
  ready(ghost)
  wrong_sector(ghost)
  for i in range(11, 16):
    result = sample(ghost, i * .01, i, oem=200.0, hrr=215.3, hrr_rate=500.0)
  assert result.mode == 'trustedHrr'
  assert result.reason == 'outsideEnvelope'
  assert result.sector_offset == 0.0
  assert result.angle == pytest.approx(215.3)


def test_oem_loss_keeps_resolved_hrr_but_cannot_start_it():
  ghost = HrrAngleShadow()
  ready(ghost)
  result = sample(ghost, .10, 5, oem_updated=False)
  assert result.mode == 'trustedHrr'
  ghost = HrrAngleShadow()
  for i in range(10):
    result = sample(ghost, i * .01, i, oem_updated=False)
  assert result.mode == 'unavailable'
  assert result.reason == 'staleOem'


def test_recovery_needs_fresh_samples_not_repeated_valid_packets():
  ghost = HrrAngleShadow()
  ready(ghost)
  sample(ghost, .05, 5, valid=False, wrap=True)
  for i in range(6, 10):
    assert sample(ghost, i * .01, i).mode != 'trustedHrr'
  assert sample(ghost, .10, 10).mode == 'trustedHrr'
  result = sample(ghost, .17, 10)
  assert result.mode != 'trustedHrr'
  assert result.hrr_age == pytest.approx(.07)


def test_counter_wrap_progresses_and_long_runs_do_not_overflow_log_field():
  ghost = HrrAngleShadow()
  for i in range(1000):
    result = sample(ghost, i * .01, i % 256)
  assert result.mode == 'trustedHrr'
  assert result.recovery_samples == 5


def test_counter_reset_requires_requalification():
  ghost = HrrAngleShadow()
  ready(ghost)
  result = sample(ghost, .05, 0)
  assert result.mode != 'trustedHrr'
  assert result.reason == 'counterDiscontinuity'
  for i in range(1, 5):
    result = sample(ghost, .05 + i * .01, i)
  assert result.mode == 'trustedHrr'


def test_batched_sequential_packets_are_not_a_counter_reset():
  ghost = HrrAngleShadow()
  ready(ghost)
  assert sample(ghost, .06, 6, hrr_sample_count=2).mode == 'trustedHrr'


@pytest.mark.parametrize('changes', [dict(oem_updated=False), dict(hrr_updated=False), dict(hrr=float('nan')),
                                     dict(oem=float('nan'))])
def test_missing_or_nonfinite_startup_is_unavailable(changes):
  result = sample(HrrAngleShadow(), 0., 0, **changes)
  assert not result.usable
  assert result.would_disable


def make_state():
  cp = car.CarParams.new_message()
  cp.carFingerprint = CAR.LEXUS_LS600h
  cp.wheelSpeedFactor = 1.0
  state = CarState(cp, custom.FrogPilotCarParams.new_message())
  def parser():
    return SimpleNamespace(vl=defaultdict(lambda: defaultdict(int)),
                           ts_nanos=defaultdict(lambda: defaultdict(int)),
                           vl_all=defaultdict(lambda: defaultdict(list)))
  pt, body, hrr = parser(), parser(), parser()
  hrr.vl['HRR_TrueAngleStatus'].update(True_Steering_Angle=12.3, True_Steering_Rate=0., True_Angle_Counter=1,
                                     True_Angle_Valid=1, True_Angle_Initialized=1,
                                     True_Angle_Resolver_Valid=1, True_Angle_Calibrated=1)
  hrr.vl_all['HRR_TrueAngleStatus']['True_Angle_Counter'] = [1]
  pt.vl['STEER_ANGLE_SENSOR']['STEER_ANGLE'] = 19.5
  pt.ts_nanos['STEER_ANGLE_SENSOR']['STEER_ANGLE'] = 1
  return state, pt, body, hrr


def step(state, pt, body, hrr, monkeypatch, i, **status):
  """Advance one control cycle with a fresh OEM sample and HRR counter."""
  monkeypatch.setattr('openpilot.selfdrive.car.toyota.carstate.time.monotonic', lambda: i * .01)
  pt.ts_nanos['STEER_ANGLE_SENSOR']['STEER_ANGLE'] = i + 1
  hrr.vl['HRR_TrueAngleStatus'].update(True_Angle_Counter=i, **status)
  hrr.vl_all['HRR_TrueAngleStatus']['True_Angle_Counter'] = [i]
  return state.update_ls600h(pt, body, hrr, shadow_lat_active=True)


def test_observer_failure_keeps_previous_steering_path():
  original, pt, body, hrr = make_state()
  broken, _, _, _ = make_state()
  def fail(*args, **kwargs):
    raise RuntimeError('test observer failure')
  broken.ls600h_angle_shadow.update = fail
  expected, _ = original.update_ls600h(pt, body, hrr)
  actual, fp = broken.update_ls600h(pt, body, hrr)
  assert actual.to_dict() == expected.to_dict()
  assert actual.steeringAngleDeg == pytest.approx(12.3)
  assert not actual.steerFaultTemporary
  assert str(fp.ls600hAngleShadow.reason) == 'observerError'


def test_observer_failure_mid_drive_reverts_to_previous_path(monkeypatch):
  state, pt, body, hrr = make_state()
  for i in range(10):
    step(state, pt, body, hrr, monkeypatch, i)
  def fail(*args, **kwargs):
    raise RuntimeError('test observer failure')
  state.ls600h_angle_shadow.update = fail
  live, fp = step(state, pt, body, hrr, monkeypatch, 10, True_Angle_Valid=0, True_Angle_Resolver_Valid=0)
  assert live.steeringAngleDeg == 19.5
  assert str(fp.ls600hAngleShadow.reason) == 'observerError'


def test_unusable_keeps_previous_path_without_new_faults():
  state, pt, body, hrr = make_state()
  live, fp = state.update_ls600h(pt, body, hrr, shadow_lat_active=True)
  assert not fp.ls600hAngleShadow.usable
  assert live.steeringAngleDeg == pytest.approx(12.3)
  assert not live.steerFaultTemporary
  assert fp.ls600hAngleShadow.wouldDisableLateral
  # Demonstrate serialization of the added diagnostics at the normal publisher boundary.
  with custom.FrogPilotCarState.from_bytes(fp.to_bytes()) as decoded:
    assert decoded.ls600hAngleShadow.version == HrrAngleShadow.VERSION
    assert decoded.ls600hAngleShadow.latActive


def test_dropout_uses_aligned_oem_instead_of_raw_oem(monkeypatch):
  state, pt, body, hrr = make_state()
  for i in range(25):
    if i >= 10:
      hrr.vl['HRR_TrueAngleStatus'].update(True_Angle_Valid=0, True_Angle_Initialized=0, True_Angle_Wrap_Ambiguous=1)
    live, fp = step(state, pt, body, hrr, monkeypatch, i)
  assert live.steeringAngleDeg == pytest.approx(12.3)
  assert live.steeringRateDeg == pytest.approx(state.ls600h_steering_rate.x)
  assert not live.steerFaultTemporary
  assert fp.ls600hAngleShadow.liveAngleDeg == 19.5
  assert str(fp.ls600hAngleShadow.mode) == 'alignedOem'


def test_wrong_sector_from_firmware_is_corrected_in_carstate(monkeypatch):
  state, pt, body, hrr = make_state()
  for i in range(10):
    step(state, pt, body, hrr, monkeypatch, i)
  step(state, pt, body, hrr, monkeypatch, 10, True_Angle_Valid=0, True_Angle_Wrap_Ambiguous=1)
  hrr.vl['HRR_TrueAngleStatus'].update(True_Angle_Valid=1, True_Angle_Wrap_Ambiguous=0)
  for i in range(11, 17):
    live, fp = step(state, pt, body, hrr, monkeypatch, i, True_Steering_Angle=34.8, True_Steering_Rate=4.0)
  assert str(fp.ls600hAngleShadow.mode) == 'sectorCorrectedHrr'
  assert live.steeringAngleDeg == pytest.approx(12.3)
  assert live.steeringRateDeg == pytest.approx(4.0)
  assert fp.ls600hAngleShadow.liveAngleDeg == pytest.approx(34.8)


def test_corrected_output_does_not_feed_previous_path_state(monkeypatch):
  state, pt, body, hrr = make_state()
  for i in range(10):
    step(state, pt, body, hrr, monkeypatch, i)
  assert state.ls600h_last_valid_hrr_angle == pytest.approx(12.3)
  step(state, pt, body, hrr, monkeypatch, 10, True_Angle_Valid=0, True_Angle_Wrap_Ambiguous=1)
  hrr.vl['HRR_TrueAngleStatus'].update(True_Angle_Valid=1, True_Angle_Wrap_Ambiguous=0)
  step(state, pt, body, hrr, monkeypatch, 11, True_Steering_Angle=34.8)
  assert state.ls600h_last_valid_hrr_angle == pytest.approx(34.8)


def test_export_actual_recorded_ghost_decisions(tmp_path):
  path = tmp_path / 'rlog'
  with path.open('wb') as f:
    for i in range(2):
      msg = log.Event.new_message()
      msg.logMonoTime = 1_000_000_000 + i * 10_000_000
      d = msg.init('frogpilotCarState').ls600hAngleShadow
      d.version = 1
      d.latActive = True
      d.mode, d.reason = 'unavailable', 'latched'
      d.wouldDisableLateral = True
      d.liveAngleDeg, d.angleDeg = 19.5, 9.2
      f.write(msg.to_bytes())
  output = tmp_path / 'ghost.csv'
  summary = export_recorded([str(path)], output)
  assert summary['recorded_shadow_samples'] == 2
  assert summary['would_disable_s'] == pytest.approx(.01)
  with output.open() as f:
    rows = list(csv.DictReader(f))
  assert float(rows[0]['liveAngleDeg']) == 19.5
  assert float(rows[0]['angleDeg']) == pytest.approx(9.2)
