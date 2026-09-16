from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import pytest

from openpilot.tools.teletyped import drive_stats


BASE_TIME_NS = 100 * 1_000_000_000


class FakeMessage(SimpleNamespace):
  def __init__(self, message_type: str, timestamp_ns: int, payload: Any) -> None:
    super().__init__(logMonoTime=timestamp_ns, **{message_type: payload})
    self.message_type = message_type

  def which(self) -> str:
    return self.message_type


def message(message_type: str, seconds: float, **payload: Any) -> FakeMessage:
  return FakeMessage(message_type, BASE_TIME_NS + int(seconds * 1e9), SimpleNamespace(**payload))


def can_message(seconds: float, *frames: tuple[int, int, bytes]) -> FakeMessage:
  payload = [SimpleNamespace(src=source, address=address, dat=data) for source, address, data in frames]
  return FakeMessage("can", BASE_TIME_NS + int(seconds * 1e9), payload)


def odometer_frame(km: float) -> tuple[int, int, bytes]:
  raw = round(km * 10)
  data = bytes([0, 0, 0, 0, 0, raw & 0xFF, (raw >> 8) & 0xFF, (raw >> 16) & 0xFF])
  return 0, 1264, data


def test_discover_drive_logs_groups_and_orders_segments(tmp_path) -> None:
  for segment, filename in ((2, "qlog"), (0, "rlog.bz2"), (1, "rlog")):
    segment_dir = tmp_path / f"2026-09-07--12-00-00--{segment}"
    segment_dir.mkdir()
    (segment_dir / filename).write_bytes(b"log")
  ignored = tmp_path / "not-a-route"
  ignored.mkdir()
  (ignored / "rlog").write_bytes(b"ignored")

  drives = drive_stats.discover_drive_logs(str(tmp_path))

  assert len(drives) == 1
  assert drives[0].name == "2026-09-07--12-00-00"
  assert [path.rsplit("--", 1)[-1].split("/", 1)[0] for path in drives[0].paths] == ["0", "1", "2"]


def test_i30_legacy_stats_match_engagement_and_torque_delta() -> None:
  analyzer = drive_stats.DriveAnalyzer("2026-09-07--12-00-00", "DONGLE")
  messages = [
    message(
      "initData",
      0,
      params=SimpleNamespace(
        entries=[
          SimpleNamespace(key="TRQISteeringToggle", value=b"0"),
        ]
      ),
    ),
    message("carParams", 0, carName="i30", carFingerprint="HYUNDAI I30 GD 2014", openpilotLongitudinalControl=False),
    message("controlsState", 0, active=False),
    can_message(0.1, odometer_frame(100.0)),
    message("controlsState", 1, active=True),
    message("carState", 1.1, vEgo=20.0, steeringTorque=-0.2, steeringTorqueEps=0.0, steeringPressed=False),
    message("controlsState", 2, active=True),
    message("controlsState", 3, active=True),
    message("controlsState", 4, active=True),
    can_message(4.1, odometer_frame(100.1)),
    message("controlsState", 5, active=True),
    message("carState", 5.1, vEgo=20.0, steeringTorque=-3.4, steeringTorqueEps=0.0, steeringPressed=False),
    can_message(5.2, (1, 559, bytes(8))),
    can_message(5.3, odometer_frame(100.2)),
    message("controlsState", 6, active=False),
  ]

  analyzer.process_segment(messages)
  result = analyzer.result()

  assert result is not None
  assert result["vehicle_profile"] == "hyundai_i30_2014"
  assert result["active_time"] == 5 * 1_000_000_000
  assert result["engagement_pct"] == pytest.approx(83.33, abs=0.01)
  assert result["odo_distance"] == pytest.approx(0.2)
  assert result["engaged_distance"] == pytest.approx(0.2)
  assert result["steer_intervention_count"] == 1
  assert result["steer_intervention_source"] == "torque_delta"
  assert result["speed_buckets"]["road"]["steer_intervention_count"] == 1
  assert result["raw_disengagement_count"] == 1
  assert result["disengagement_count"] == 0
  assert result["disengagements_per_100km"] == 0.0
  assert result["disengagements_per_drive_hour"] is None
  assert result["disengagement_corrections"]["manual_shutdown_removed"] == 1


def test_disengagement_rates_exclude_only_final_unmatched_transition() -> None:
  analyzer = drive_stats.DriveAnalyzer(
    "2026-09-07--12-30-00",
    "DONGLE",
    drive_stats.HyundaiI30StatsProfile(),
  )
  analyzer.process_segment(
    [
      message("controlsState", 0, active=True),
      message("carState", 0.1, vEgo=20.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      can_message(0.2, odometer_frame(100.0)),
      message("controlsState", 1, active=False),
      message("carState", 1.1, vEgo=20.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      can_message(1.2, odometer_frame(100.5)),
      message("controlsState", 2, active=True),
      message("carState", 2.1, vEgo=20.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      can_message(2.2, odometer_frame(101.0)),
      message("controlsState", 3, active=True),
      message("carState", 3.1, vEgo=20.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      can_message(3.2, odometer_frame(101.5)),
      message("controlsState", 4, active=False),
      message("carState", 4.1, vEgo=20.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      can_message(4.2, odometer_frame(102.0)),
    ]
  )

  result = analyzer.result()

  assert result is not None
  assert result["raw_disengagement_count"] == 2
  assert result["disengagement_count"] == 1
  assert result["disengagements_per_100km"] == 50.0
  assert result["disengagements_per_drive_hour"] == 900.0
  assert result["disengagement_corrections"] == {
    "version": 1,
    "raw_count": 2,
    "corrected_count": 1,
    "manual_shutdown_removed": 1,
  }
  assert result["speed_buckets"]["road"]["raw_disengagement_count"] == 2
  assert result["speed_buckets"]["road"]["disengagement_count"] == 1
  assert result["speed_buckets"]["road"]["manual_shutdown_removed"] == 1
  assert result["speed_buckets"]["road"]["disengagements_per_100km"] == 1250.0
  assert result["speed_buckets"]["road"]["disengagements_per_drive_hour"] == 900.0


def test_speed_buckets_track_time_distance_and_shutdown_correction() -> None:
  analyzer = drive_stats.DriveAnalyzer(
    "2026-09-07--12-45-00",
    "DONGLE",
    drive_stats.HyundaiI30StatsProfile(),
  )
  analyzer.process_segment(
    [
      message("controlsState", 0, active=True),
      message("carState", 0, vEgo=10.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      message("carState", 1, vEgo=15.3, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      message("carState", 2, vEgo=25.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      message("carState", 3, vEgo=25.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
      message("controlsState", 3.1, active=False),
    ]
  )

  result = analyzer.result()

  assert result is not None
  assert result["speed_bucket_version"] == 1
  assert result["distance_source"] == "integrated_v_ego"
  assert result["odo_distance"] == pytest.approx(0.05)
  assert result["speed_buckets"]["city"]["time_ns"] == 1_000_000_000
  assert result["speed_buckets"]["city"]["distance_km_raw"] == pytest.approx(0.01)
  assert result["speed_buckets"]["road"]["time_ns"] == 1_000_000_000
  assert result["speed_buckets"]["road"]["distance_km_raw"] == pytest.approx(0.0153)
  assert result["speed_buckets"]["highway"]["time_ns"] == 1_000_000_000
  assert result["speed_buckets"]["highway"]["distance_km_raw"] == pytest.approx(0.025)
  assert result["speed_buckets"]["highway"]["raw_disengagement_count"] == 1
  assert result["speed_buckets"]["highway"]["disengagement_count"] == 0
  assert result["speed_buckets"]["highway"]["manual_shutdown_removed"] == 1


def test_upload_metadata_comes_from_route_messages() -> None:
  analyzer = drive_stats.DriveAnalyzer("00000489--d20ab4f43f", "DONGLE")
  analyzer.process_segment(
    [
      message(
        "initData",
        0,
        version="0.9.7",
        gitBranch="feature/stats",
        gitCommit="0123456789abcdef",
        gitCommitDate="2026-09-07T10:00:00Z",
        dirty=True,
        deviceType="tici",
        wallTimeNanos=1_788_777_600_000_000_000,
        params=SimpleNamespace(entries=[]),
      ),
      message("carParams", 0, carName="i30", carFingerprint="HYUNDAI_I30_GD_2014", openpilotLongitudinalControl=False),
    ]
  )

  result = analyzer.result()

  assert result is not None
  assert result["car_name"] == "i30"
  assert result["car_fingerprint"] == "HYUNDAI_I30_GD_2014"
  assert result["git_branch"] == "feature/stats"
  assert result["git_commit"] == "0123456789abcdef"
  assert result["git_commit_date"] == "2026-09-07T10:00:00Z"
  assert result["git_dirty"] is True
  assert result["device_type"] == "tici"
  assert result["recorded_at"] == "2026-09-07T10:40:00+00:00"
  assert result["segment_count"] == 1


def test_i30_trqi_uses_hardware_steering_pressed() -> None:
  analyzer = drive_stats.DriveAnalyzer("2026-09-07--13-00-00", "DONGLE")
  analyzer.process_segment(
    [
      message(
        "initData",
        0,
        params=SimpleNamespace(
          entries=[
            SimpleNamespace(key="TRQISteeringToggle", value=b"1"),
          ]
        ),
      ),
      message("carParams", 0, carName="i30", carFingerprint="HYUNDAI I30 GD 2014", openpilotLongitudinalControl=False),
      message("controlsState", 0, active=True),
      message("controlsState", 1, active=True),
      message("controlsState", 2, active=True),
      message("controlsState", 3, active=True),
      message("controlsState", 4, active=True),
      message("carState", 4.1, vEgo=20.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=True),
      message("carState", 4.2, vEgo=20.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=True),
      message("carState", 4.3, vEgo=20.0, steeringTorque=0.0, steeringTorqueEps=0.0, steeringPressed=False),
    ]
  )

  result = analyzer.result()

  assert result is not None
  assert result["steer_intervention_count"] == 1
  assert result["steer_intervention_source"] == "steering_pressed"
  assert result["speed_buckets"]["road"]["steer_intervention_count"] == 1


def test_logged_non_i30_metadata_overrides_cached_i30_profile() -> None:
  analyzer = drive_stats.DriveAnalyzer(
    "2026-09-07--14-00-00",
    "DONGLE",
    drive_stats.HyundaiI30StatsProfile(),
  )
  analyzer.process_segment(
    [
      message("carParams", 0, carName="toyota", carFingerprint="TOYOTA COROLLA TSS2 2019", openpilotLongitudinalControl=False),
    ]
  )

  assert analyzer.result() is None


def test_drive_stats_step_persists_payload_for_upload_retry(monkeypatch: pytest.MonkeyPatch) -> None:
  drive = drive_stats.DriveLogSet("2026-09-07--15-00-00", ("rlog",), "fingerprint")
  state: dict[str, Any] = {"version": drive_stats.DRIVE_STATS_SCHEMA_VERSION, "drives": {}}
  saved: list[dict[str, Any]] = []
  attempts = 0

  def save(updated: dict[str, Any]) -> None:
    saved.append({"version": updated["version"], "drives": {k: dict(v) for k, v in updated["drives"].items()}})

  def upload(stats: dict[str, Any], headers: dict[str, str]) -> bool:
    nonlocal attempts
    attempts += 1
    if attempts == 1:
      raise OSError("offline")
    return True

  monkeypatch.setattr(drive_stats, "discover_drive_logs", lambda: [drive])
  monkeypatch.setattr(drive_stats, "DRIVE_STATS_RETRY_INTERVAL", 0)
  monkeypatch.setattr(drive_stats, "_load_state", lambda: state)
  monkeypatch.setattr(drive_stats, "_save_state", save)
  monkeypatch.setattr(drive_stats, "_current_vehicle_profile", lambda: drive_stats.HyundaiI30StatsProfile())
  monkeypatch.setattr(drive_stats, "analyze_drive", lambda *args: {"drive": drive.name, "device_id": "DONGLE"})
  monkeypatch.setattr(drive_stats, "upload_drive_stats", upload)
  monkeypatch.setattr(drive_stats, "capture_exception", lambda _error: None)
  monkeypatch.setattr(drive_stats, "log", lambda *args: None)

  drive_stats.drive_stats_step("DONGLE", {"X-Device-JWT": "jwt"})
  assert state["drives"][drive.name]["status"] == "pending_upload"
  assert state["drives"][drive.name]["stats"]["drive"] == drive.name

  drive_stats.drive_stats_step("DONGLE", {"X-Device-JWT": "jwt"})
  assert state["drives"][drive.name]["status"] == "uploaded"
  assert "stats" not in state["drives"][drive.name]
  assert attempts == 2
  assert saved
