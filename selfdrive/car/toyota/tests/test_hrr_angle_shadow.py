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
  args = dict(now=now, oem=19.5, oem_rate=0.0, oem_updated=True, hrr=9.2, hrr_rate=0.0,
              hrr_updated=True, counter=counter, valid=True, wrap=False, lat_active=False)
  args.update(changes)
  return observer.update(**args)


def ready(observer):
  for i in range(5):
    result = sample(observer, i * .01, i)
  assert result.mode == 'trustedHrr'


def test_real_fallback_sequence_preserves_reference_and_tracks_motion():
  ghost = HrrAngleShadow()
  ready(ghost)
  assert sample(ghost, .05, 5, valid=False, wrap=True, lat_active=True).mode == 'hold'
  result = sample(ghost, .10, 10, valid=False, wrap=True, lat_active=True)
  assert result.mode == 'alignedOem'
  assert result.angle == pytest.approx(9.2)
  moved = sample(ghost, .12, 12, oem=21.0, valid=False, wrap=True, lat_active=True)
  assert moved.angle == pytest.approx(10.7)
  assert moved.angle != 21.0


def test_valid_sector_jump_does_not_redefine_anchor():
  ghost = HrrAngleShadow()
  ready(ghost)
  result = sample(ghost, .05, 5, hrr=30.63, lat_active=True)
  assert result.reason == 'inconsistent'
  assert result.angle == pytest.approx(9.2)
  assert result.offset == pytest.approx(-10.3)
  assert result.mode == 'alignedOem'


def test_expired_bridge_latches_until_inactive_qualified_recovery():
  ghost = HrrAngleShadow()
  ready(ghost)
  assert sample(ghost, .30, 30, valid=False, wrap=True, lat_active=True).would_disable
  for i in range(31, 38):
    result = sample(ghost, i * .01, i, lat_active=True)
    assert result.would_disable
    assert result.reason == 'latched'
  result = sample(ghost, .38, 38)
  assert result.mode == 'trustedHrr'
  assert not result.would_disable


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


@pytest.mark.parametrize('changes', [dict(oem_updated=False), dict(hrr_updated=False), dict(hrr=float('nan'))])
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
  hrr.vl['HRR_TrueAngleStatus'].update(True_Steering_Angle=9.2, True_Steering_Rate=0., True_Angle_Counter=1,
                                     True_Angle_Valid=1, True_Angle_Initialized=1,
                                     True_Angle_Resolver_Valid=1, True_Angle_Calibrated=1)
  hrr.vl_all['HRR_TrueAngleStatus']['True_Angle_Counter'] = [1]
  pt.vl['STEER_ANGLE_SENSOR']['STEER_ANGLE'] = 19.5
  pt.ts_nanos['STEER_ANGLE_SENSOR']['STEER_ANGLE'] = 1
  return state, pt, body, hrr


def test_shadow_failure_cannot_change_live_carstate():
  original, pt, body, hrr = make_state()
  broken, _, _, _ = make_state()
  def fail(*args, **kwargs):
    raise RuntimeError('test observer failure')
  broken.ls600h_angle_shadow.update = fail
  expected, _ = original.update_ls600h(pt, body, hrr)
  actual, fp = broken.update_ls600h(pt, body, hrr)
  assert actual.to_dict() == expected.to_dict()
  assert actual.steeringAngleDeg == pytest.approx(9.2)
  assert not actual.steerFaultTemporary
  assert str(fp.ls600hAngleShadow.reason) == 'observerError'


def test_proposed_unavailability_only_changes_shadow_message():
  state, pt, body, hrr = make_state()
  live, fp = state.update_ls600h(pt, body, hrr, shadow_lat_active=True)
  assert live.steeringAngleDeg == pytest.approx(9.2)
  assert not live.steerFaultTemporary
  assert fp.ls600hAngleShadow.wouldDisableLateral
  # Demonstrate serialization of the added diagnostics at the normal publisher boundary.
  with custom.FrogPilotCarState.from_bytes(fp.to_bytes()) as decoded:
    assert decoded.ls600hAngleShadow.version == HrrAngleShadow.VERSION
    assert decoded.ls600hAngleShadow.latActive


def test_live_fallback_jump_is_logged_but_only_shadow_corrects_it(monkeypatch):
  state, pt, body, hrr = make_state()
  status = hrr.vl['HRR_TrueAngleStatus']
  for i in range(25):
    monkeypatch.setattr('openpilot.selfdrive.car.toyota.carstate.time.monotonic', lambda i=i: i * .01)
    pt.ts_nanos['STEER_ANGLE_SENSOR']['STEER_ANGLE'] = i + 1
    status['True_Angle_Counter'] = i
    hrr.vl_all['HRR_TrueAngleStatus']['True_Angle_Counter'] = [i]
    if i >= 10:
      status.update(True_Angle_Valid=0, True_Angle_Initialized=0, True_Angle_Wrap_Ambiguous=1)
    live, fp = state.update_ls600h(pt, body, hrr, shadow_lat_active=i >= 5)
  assert live.steeringAngleDeg == 19.5
  assert not live.steerFaultTemporary
  assert fp.ls600hAngleShadow.liveAngleDeg == 19.5
  assert fp.ls600hAngleShadow.angleDeg == pytest.approx(9.2)
  assert str(fp.ls600hAngleShadow.mode) == 'alignedOem'


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
