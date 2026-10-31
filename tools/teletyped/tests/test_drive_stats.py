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


def test_motorhome_uses_integrated_distance_and_leaves_steering_interventions_blank() -> None:
  analyzer = drive_stats.DriveAnalyzer("2026-09-07--13-30-00", "DONGLE")
  analyzer.process_segment(
    [
      message("carParams", 0, carName="motorhome", carFingerprint="MOTORHOME", openpilotLongitudinalControl=False),
      message("controlsState", 0, active=True),
      message("carState", 0, vEgo=20.0, steeringTorque=100.0, steeringTorqueEps=-100.0, steeringPressed=True),
      message("carState", 1, vEgo=20.0, steeringTorque=100.0, steeringTorqueEps=-100.0, steeringPressed=True),
      message("controlsState", 1, active=False),
    ]
  )

  result = analyzer.result()

  assert result is not None
  assert result["vehicle_profile"] == "motorhome_ssc"
  assert result["distance_source"] == "integrated_v_ego"
  assert result["odo_distance"] == pytest.approx(0.02)
  assert result["engaged_distance"] == pytest.approx(0.02)
  assert result["steer_intervention_count"] is None
  assert result["steer_interventions_per_100km"] is None
  assert result["steer_intervention_source"] is None
  assert result["steering_mode"] == "legacy_ssc"
  assert result["schema_version"] == 3
  assert result["speed_buckets"]["road"]["steer_intervention_count"] is None
  assert result["speed_buckets"]["road"]["steer_interventions_per_100km"] is None
  assert result["non_intersection_disengagement_count"] is None
  assert result["non_intersection_detection"] is None
  assert result["speed_buckets"]["road"]["non_intersection_disengagement_count"] is None


def test_logged_vehicle_overrides_cached_motorhome_profile() -> None:
  profile = drive_stats.select_vehicle_profile("motorhome", "MOTORHOME")
  assert profile is not None
  assert drive_stats.select_vehicle_profile("toyota", "LEXUS_LS600h") is None
  assert drive_stats.select_vehicle_profile("i30", "HYUNDAI I30 GD 2014") is None
  analyzer = drive_stats.DriveAnalyzer("route", "device", profile)
  analyzer.process_segment([message("carParams", 0, carName="other", carFingerprint="other")])
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
  monkeypatch.setattr(drive_stats, "_current_vehicle_profile", lambda: drive_stats.select_vehicle_profile("motorhome", "MOTORHOME"))
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


@pytest.fixture
def stats_worker(tmp_path, monkeypatch: pytest.MonkeyPatch):
  realdata = tmp_path / "realdata"
  drive_name = "0000011a--d9d5ec8358"
  for segment in range(3):
    directory = realdata / f"{drive_name}--{segment}"
    directory.mkdir(parents=True)
    (directory / "rlog").write_bytes(b"log")

  discover = drive_stats.discover_drive_logs
  monkeypatch.setattr(drive_stats, "discover_drive_logs", lambda: discover(str(realdata)))
  monkeypatch.setattr(drive_stats, "_DRIVE_STATS_STATE_PATH", str(tmp_path / "stats-state.json"))
  monkeypatch.setattr(drive_stats, "DRIVE_STATS_ENABLED", True)
  monkeypatch.setattr(drive_stats, "_current_vehicle_profile", lambda: None)
  analyzed = []
  posted = []
  response = SimpleNamespace(status_code=201, raise_for_status=lambda: None)

  def analyze(*args):
    analyzed.append(args)
    return {"device_id": "DONGLE", "drive": drive_name, "schema_version": drive_stats.DRIVE_STATS_SCHEMA_VERSION}

  def post(url, **kwargs):
    posted.append((url, kwargs))
    return response

  def fail_on_exception(error: BaseException) -> None:
    pytest.fail(str(error))

  monkeypatch.setattr(drive_stats, "analyze_drive", analyze)
  monkeypatch.setattr(drive_stats, "http_post", post)
  monkeypatch.setattr(drive_stats, "capture_exception", fail_on_exception)
  monkeypatch.setattr(drive_stats, "log", lambda *args: None)
  return SimpleNamespace(realdata=realdata, drive_name=drive_name, analyzed=analyzed, posted=posted, response=response)


@pytest.mark.parametrize("status_code", [201, 409])
@pytest.mark.parametrize("change", ["delete_segment", "compress_log"])
def test_uploaded_drive_stays_complete_when_logs_change(stats_worker, status_code: int, change: str) -> None:
  worker = stats_worker
  worker.response.status_code = status_code
  drive_stats.drive_stats_step("DONGLE", {})
  uploaded = drive_stats._load_state()["drives"][worker.drive_name]
  assert uploaded["status"] == "uploaded"
  assert len(worker.analyzed) == 1
  assert len(worker.posted) == 1

  log_path = worker.realdata / f"{worker.drive_name}--0" / "rlog"
  if change == "delete_segment":
    log_path.unlink()
    log_path.parent.rmdir()
  else:
    log_path.rename(log_path.with_name("rlog.bz2"))
  assert drive_stats.discover_drive_logs()[0].fingerprint != uploaded["fingerprint"]

  drive_stats.drive_stats_step("DONGLE", {})

  assert len(worker.analyzed) == 1
  assert len(worker.posted) == 1
  assert drive_stats._load_state()["drives"][worker.drive_name] == uploaded


def test_schema_upgrade_reprocesses_a_previously_uploaded_drive(stats_worker) -> None:
  worker = stats_worker
  drive = drive_stats.discover_drive_logs()[0]
  drive_stats._save_state({
    "version": drive_stats.DRIVE_STATS_SCHEMA_VERSION - 1,
    "drives": {drive.name: {"fingerprint": drive.fingerprint, "status": "uploaded"}},
  })

  drive_stats.drive_stats_step("DONGLE", {})

  assert len(worker.analyzed) == 1
  assert len(worker.posted) == 1
  state = drive_stats._load_state()
  assert state["version"] == drive_stats.DRIVE_STATS_SCHEMA_VERSION
  assert state["drives"][drive.name]["status"] == "uploaded"


@pytest.mark.parametrize("status", ["unsupported", "error"])
def test_unfinished_drive_is_reconsidered_when_its_logs_change(stats_worker, status: str) -> None:
  worker = stats_worker
  drive = drive_stats.discover_drive_logs()[0]
  drive_stats._save_state({
    "version": drive_stats.DRIVE_STATS_SCHEMA_VERSION,
    "drives": {drive.name: {"fingerprint": drive.fingerprint, "status": status}},
  })
  drive_stats.drive_stats_step("DONGLE", {})
  assert not worker.analyzed
  assert not worker.posted

  log_path = worker.realdata / f"{drive.name}--0" / "rlog"
  log_path.rename(log_path.with_name("rlog.bz2"))
  drive_stats.drive_stats_step("DONGLE", {})

  assert len(worker.analyzed) == 1
  assert len(worker.posted) == 1
  assert drive_stats._load_state()["drives"][drive.name]["status"] == "uploaded"
