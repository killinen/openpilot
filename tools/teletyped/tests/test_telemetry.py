from __future__ import annotations

import os
import gzip
import json
from types import SimpleNamespace

import pytest

from openpilot.tools.teletyped import telemetry
from openpilot.tools.teletyped.collectors import enabled_collectors
from openpilot.tools.teletyped.collectors.hrr_temperature import HrrTemperatureCollector
from openpilot.tools.teletyped.route_logs import DriveLogSet, discover_drive_logs
from openpilot.tools.teletyped.tests.test_drive_stats import can_message, message


def hrr(temp: int = 50, counter: int = 1, valid: bool = True, bus: int = 2, length: int = 8):
  return bus, 0x638, (temp.to_bytes(2, "little", signed=True) + bytes([valid, counter]) + bytes(4))[:length]


def speed(kph: float = 90, bus: int = 0):
  return bus, 0xB4, bytes(5) + round(kph * 100).to_bytes(2, "big") + b"\x00"


def ambient(temp: int = 20, bus: int = 1):
  return bus, 0x3B0, bytes([0, 0, 0, temp + 49, 0, 0])


def hrr_rows(result):
  return [row for row in result["samples"] if row["metric"] == "hrr"]


def test_hrr_decode_matches_tracker_and_uses_configured_receive_buses():
  collector = HrrTemperatureCollector()
  collector.observe(can_message(0, speed(200, 1), ambient(99, 0), hrr(100, bus=130)), 0, 0)
  assert hrr_rows(collector.result()) == []
  collector.observe(can_message(1, hrr(-5, length=4), speed(), ambient()), 1, 0)
  row = hrr_rows(collector.result())[0]
  assert row["value"] == -5
  streams = {sample["metric"]: sample for sample in collector.result()["samples"]}
  assert streams["speed"]["value"] == 90
  assert streams["ambient"]["value"] == 20
  assert streams["ambient"]["raw"] == 69
  assert row["time_ns"] == 101_000_000_000
  assert collector.result()["signals"]["ambient"]["provisional"] is True


def test_counters_validity_and_freshness_continue_across_segments():
  collector = HrrTemperatureCollector()
  collector.observe(can_message(0, speed(), ambient(), hrr(counter=255)), 0, 0)
  collector.observe(can_message(1, hrr(counter=255)), 1, 1)
  collector.observe(can_message(3, hrr(counter=0)), 3, 1)
  collector.observe(can_message(4, hrr(counter=0, valid=False)), 4, 1)
  collector.observe(can_message(61, hrr(counter=1)), 61, 2)
  rows = hrr_rows(collector.result())
  assert len(rows) == 4
  assert collector.result()["repeated_temperature_reports"] == 1
  assert rows[1]["counter"] == 0
  assert rows[2]["value"] is None
  assert rows[3]["time_s"] == 61
  assert collector.result()["signals"]["speed"]["max_age_s"] == 2
  assert collector.result()["signals"]["ambient"]["max_age_s"] == 60


def test_malformed_frames_are_counted_and_ignored():
  collector = HrrTemperatureCollector()
  collector.observe(can_message(0, hrr(length=3), (0, 0xB4, bytes(6)), (1, 0x3B0, bytes(3)), hrr()), 0, 0)
  assert collector.result()["malformed_frames"] == 3
  assert not any(row["metric"] == "speed" for row in collector.result()["samples"])


def test_optional_activation(monkeypatch):
  monkeypatch.setenv("TELETYPED_HRR_TEMPERATURE", "off")
  assert enabled_collectors() == []
  monkeypatch.setenv("TELETYPED_HRR_TEMPERATURE", "1")
  assert len(enabled_collectors()) == 1
  assert not HrrTemperatureCollector().matches("i30", "HYUNDAI I30 GD 2014")


def test_discovery_excludes_qlogs_for_can_collection(tmp_path):
  for number, filename in ((0, "rlog"), (1, "qlog"), (2, "rlog.bz2")):
    directory = tmp_path / f"00000004--52b0dd2646--{number}"
    directory.mkdir()
    (directory / filename).write_bytes(b"log")
  assert len(discover_drive_logs(str(tmp_path))[0].paths) == 3
  assert len(discover_drive_logs(str(tmp_path), rlogs_only=True)[0].paths) == 2


def test_analysis_batches_and_logged_vehicle_selection(monkeypatch):
  monkeypatch.setattr(telemetry, "TELEMETRY_BATCH_SIZE", 2)
  drive = DriveLogSet("00000004--52b0dd2646", ("route--0/rlog", "route--1/rlog"), "a" * 64)
  segments = {
    drive.paths[0]: [
      message("initData", 0, wallTimeNanos=1_700_000_000_000_000_000, gitCommit="abc"),
      message("carParams", 0, carName="toyota", carFingerprint="LEXUS_LS600H"),
      can_message(1, hrr(counter=1), speed(), ambient()),
      can_message(2, hrr(counter=2)),
    ],
    drive.paths[1]: [can_message(3, hrr(counter=2)), can_message(4, hrr(counter=3))],
  }
  payloads = telemetry.analyze_telemetry(drive, "DONGLE", [HrrTemperatureCollector()], lambda path, **kw: segments[path], lambda: False)
  assert len(payloads) == 3
  reports = [row for payload in payloads for row in hrr_rows(payload["data"])]
  assert [row["time_s"] for row in reports] == [1, 2, 4]
  assert reports[-1]["segment"] == 1
  assert payloads[0]["batch_count"] == 3
  assert payloads[-1]["batch_index"] == 2
  assert "coverage" in payloads[0]["data"]
  assert "coverage" not in payloads[-1]["data"]
  assert payloads[0]["recorded_at"] == "2023-11-14T22:13:20+00:00"
  segments[drive.paths[0]][1] = message("carParams", 0, carName="i30", carFingerprint="HYUNDAI I30 GD 2014")
  assert telemetry.analyze_telemetry(drive, "DONGLE", [HrrTemperatureCollector()], lambda path, **kw: segments[path], lambda: False) == []


def test_analysis_stops_when_going_onroad():
  drive = DriveLogSet("route", ("route--0/rlog",), "a" * 64)
  with pytest.raises(telemetry.AnalysisInterrupted):
    telemetry.analyze_telemetry(drive, "DONGLE", [HrrTemperatureCollector()], should_stop=lambda: True)


def test_can_before_car_params_is_collected_using_logged_identity():
  drive = DriveLogSet("route", ("route--0/rlog", "route--1/rlog"), "a" * 64)
  segments = {
    drive.paths[0]: [can_message(0, hrr(counter=1), speed(), ambient())],
    drive.paths[1]: [
      can_message(1, hrr(counter=2)),
      message("carParams", 2, carName="toyota", carFingerprint="LEXUS_LS600H"),
    ],
  }
  result = telemetry.analyze_telemetry(drive, "DONGLE", [HrrTemperatureCollector()], lambda path, **kw: segments[path], lambda: False)
  assert [row["time_s"] for row in hrr_rows(result[0]["data"])] == [0, 1]


def test_analysis_failure_has_backoff_and_retries(queue_setup, monkeypatch):
  attempts = []

  def analyze(*args):
    attempts.append(True)
    if len(attempts) == 1:
      raise OSError("temporary read failure")
    return []

  monkeypatch.setattr(telemetry, "analyze_telemetry", analyze)
  telemetry.telemetry_step("DONGLE", {})
  telemetry.telemetry_step("DONGLE", {})
  assert len(attempts) == 1
  assert next(iter(telemetry._load_state()["jobs"].values()))["status"] == "analysis_error"
  monkeypatch.setattr(telemetry.time, "time", lambda: 2000)
  telemetry.telemetry_step("DONGLE", {})
  assert len(attempts) == 2
  assert next(iter(telemetry._load_state()["jobs"].values()))["status"] == "no_data"


@pytest.fixture
def queue_setup(tmp_path, monkeypatch):
  monkeypatch.setattr(telemetry, "_TELEMETRY_STATE_PATH", str(tmp_path / "state.json"))
  monkeypatch.setattr(telemetry, "_is_onroad", lambda: False)
  monkeypatch.setattr(telemetry, "log", lambda *args: None)
  monkeypatch.setattr(telemetry, "capture_exception", lambda *args: None)
  monkeypatch.setattr(telemetry.time, "time", lambda: 1000)
  directory = tmp_path / "00000004--52b0dd2646--0"
  directory.mkdir()
  path = directory / "rlog"
  path.write_bytes(b"log")
  os.utime(path, (0, 0))
  drive = DriveLogSet("00000004--52b0dd2646", (str(path),), "a" * 64)
  monkeypatch.setattr(telemetry, "discover_drive_logs", lambda **kw: [drive])
  return drive


def test_retry_survives_restart_deleted_logs_and_resumes_batches(queue_setup, monkeypatch):
  calls = []
  analyzed = []
  payloads = [{"collector": "hrr_temperature", "batch_index": i} for i in range(2)]
  monkeypatch.setattr(telemetry, "analyze_telemetry", lambda *args: analyzed.append(True) or payloads)

  def post(url, *, data, headers, timeout):
    assert headers["Content-Encoding"] == "gzip"
    payload = json.loads(gzip.decompress(data))
    calls.append(payload["batch_index"])
    if len(calls) == 2:
      raise RuntimeError("network down")
    return SimpleNamespace(status_code=409, raise_for_status=lambda: None)

  monkeypatch.setattr(telemetry, "http_post", post)
  telemetry.telemetry_step("DONGLE", {})
  state = telemetry._load_state()
  job = next(iter(state["jobs"].values()))
  assert job["status"] == "pending_upload"
  assert job["next_batch"] == 1
  monkeypatch.setattr(telemetry, "discover_drive_logs", lambda **kw: [])
  telemetry.telemetry_step("DONGLE", {})
  assert calls == [0, 1]  # retry backoff
  monkeypatch.setattr(telemetry.time, "time", lambda: 2000)
  telemetry.telemetry_step("DONGLE", {})
  assert calls == [0, 1, 1]
  assert len(analyzed) == 1
  assert next(iter(telemetry._load_state()["jobs"].values()))["status"] == "uploaded"


def test_onroad_and_disabled_collectors_do_not_read_logs(queue_setup, monkeypatch):
  monkeypatch.setattr(telemetry, "discover_drive_logs", lambda **kw: pytest.fail("Must not read logs"))
  monkeypatch.setattr(telemetry, "_is_onroad", lambda: True)
  telemetry.telemetry_step("DONGLE", {})
  monkeypatch.setattr(telemetry, "_is_onroad", lambda: False)
  monkeypatch.setenv("TELETYPED_HRR_TEMPERATURE", "0")
  telemetry.telemetry_step("DONGLE", {})


def test_changed_source_is_not_queued(queue_setup, monkeypatch):
  monkeypatch.setattr(telemetry, "analyze_telemetry", lambda *args: [])
  scans = iter([[queue_setup], []])
  monkeypatch.setattr(telemetry, "discover_drive_logs", lambda **kw: next(scans))
  telemetry.telemetry_step("DONGLE", {})
  assert telemetry._load_state()["jobs"] == {}


def test_no_data_is_cached_and_fresh_logs_wait(queue_setup, monkeypatch):
  analyzed = []
  monkeypatch.setattr(telemetry, "analyze_telemetry", lambda *args: analyzed.append(True) or [])
  os.utime(queue_setup.paths[0], (950, 950))
  telemetry.telemetry_step("DONGLE", {})
  assert analyzed == []
  os.utime(queue_setup.paths[0], (0, 0))
  telemetry.telemetry_step("DONGLE", {})
  telemetry.telemetry_step("DONGLE", {})
  assert analyzed == [True]
  assert next(iter(telemetry._load_state()["jobs"].values()))["status"] == "no_data"


def test_companion_sampling_keeps_last_original_timestamp_per_absolute_second():
  collector = HrrTemperatureCollector()
  for seconds, kph in ((0.1, 20), (0.9, 30), (1.1, 40)):
    collector.observe(can_message(seconds, speed(kph), (1, 0x39, bytes([85]))), seconds, 0)
  samples = collector.result()["samples"]
  speeds = [row for row in samples if row["metric"] == "speed"]
  assert [(row["time_s"], row["value"]) for row in speeds] == [(0.9, 30), (1.1, 40)]
  assert speeds[0]["time_ns"] == 100_900_000_000
  assert any(row["metric"] == "engine" and row["value"] == 85 for row in samples)


def test_coverage_only_segment_is_uploaded(monkeypatch):
  drive = DriveLogSet("route", ("route--0/rlog",), "a" * 64)
  messages = [message("carParams", 0, carName="toyota", carFingerprint="LEXUS_LS600H"),
              can_message(1, (0, 0x123, bytes(8))), can_message(60, (0, 0x123, bytes(8)))]
  payloads = telemetry.analyze_telemetry(drive, "DONGLE", [HrrTemperatureCollector()], lambda *args, **kw: messages, lambda: False)
  assert len(payloads) == 1
  assert payloads[0]["data"]["samples"] == []
  assert payloads[0]["data"]["coverage"][0]["hrr_frames"] == 0
  assert payloads[0]["data"]["coverage"][0]["last_ns"] == 160_000_000_000


def test_compressed_queue_migrates_plain_json_and_retries_v1(queue_setup, monkeypatch):
  path = str(queue_setup.paths[0]) + ".state.json.gz"
  monkeypatch.setattr(telemetry, "_TELEMETRY_STATE_PATH", path)
  old = {"version": 1, "jobs": {"hrr_temperature:1:route": {
    "status": "pending_upload", "fingerprint": "a" * 64, "next_batch": 0,
    "payloads": [{"collector": "hrr_temperature", "schema_version": 1}],
  }}}
  with open(path[:-3], "w") as file:
    json.dump(old, file)
  assert telemetry._load_state() == old
  monkeypatch.setattr(telemetry, "discover_drive_logs", lambda **kw: [])
  uploaded = []
  monkeypatch.setattr(telemetry, "http_post", lambda url, **kw: uploaded.append(json.loads(gzip.decompress(kw["data"]))) or
                      SimpleNamespace(status_code=201, raise_for_status=lambda: None))
  telemetry.telemetry_step("DONGLE", {})
  assert uploaded[0]["schema_version"] == 1
  assert telemetry._load_state()["jobs"]["hrr_temperature:1:route"]["status"] == "uploaded"
  assert not os.path.exists(path[:-3])
  with open(path, "rb") as file:
    assert file.read(2) == b"\x1f\x8b"


@pytest.mark.parametrize("raw", [b"broken", b"null", b"[]", b'{"version":1,"jobs":[]}'])
def test_corrupt_queue_is_preserved_instead_of_reset(queue_setup, raw):
  path = telemetry._state_path()
  with open(path, "wb") as file:
    file.write(raw)
  with pytest.raises(ValueError):
    telemetry._load_state()
  with open(path, "rb") as file:
    assert file.read() == raw
