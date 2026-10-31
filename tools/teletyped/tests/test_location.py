import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.tools.teletyped import location

FIX_TIME = 1_760_000_000


def gps(seconds=0, *, latitude=60.1, longitude=24.9, valid=True, has_fix=True):
  payload = SimpleNamespace(latitude=latitude, longitude=longitude, unixTimestampMillis=(FIX_TIME + seconds) * 1000,
                            hasFix=has_fix, flags=0, horizontalAccuracy=3.5, speed=0)
  return SimpleNamespace(which=lambda: "gpsLocationExternal", valid=valid, gpsLocationExternal=payload)


class FakeParams:
  onroad = False
  fallback: str | None = None

  def get_bool(self, key):
    return self.onroad

  def get(self, key):
    return self.fallback


@pytest.fixture
def setup(tmp_path, monkeypatch):
  clock = [1000]
  monkeypatch.setattr(location.time, "monotonic", lambda: clock[0])
  sent, messages, reads = [], {}, []

  def reader(path):
    reads.append(path)
    yield from messages[path]

  def post(url, **kwargs):
    sent.append((url, kwargs))
    return SimpleNamespace(raise_for_status=lambda: None)

  monkeypatch.setattr(location, "http_post", post)
  params = FakeParams()
  reporter = location.LocationReporter(str(tmp_path / "logs"), str(tmp_path / "location.json"), params=params, log_reader=reader)

  def logfile(segment=0, name="qlog", age=60):
    directory = Path(reporter.root) / f"2026-10-14--12-00-00--{segment}"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / name
    path.write_bytes(b"test")
    os.utime(path, (location.time.time() - age, location.time.time() - age))
    return str(path)

  return reporter, params, clock, sent, messages, reads, logfile


def test_validity_zero_coordinates_original_time_and_finite_fields():
  fix = location.fix_from_message(gps(latitude=0, longitude=0))
  assert fix["latitude"] == fix["longitude"] == 0
  assert fix["fix_time"] == FIX_TIME
  assert fix["speed_mps"] == 0
  for message in (gps(has_fix=False), gps(valid=False), gps(latitude=float("nan")), gps(longitude=181), gps(latitude=91)):
    assert location.fix_from_message(message) is None
  future = gps()
  future.gpsLocationExternal.unixTimestampMillis = (location.time.time() + 1000) * 1000
  assert location.fix_from_message(future) is None
  legacy = gps(has_fix=False)
  legacy.gpsLocationExternal.flags = 1
  assert location.fix_from_message(legacy) is not None


def test_last_valid_report_wins_and_retries_do_not_reparse(setup):
  reporter, _, clock, sent, messages, reads, logfile = setup
  path = logfile()
  messages[path] = [gps(3), gps(1), gps(9, has_fix=False), gps(5)]
  reporter.step("device_1", {"X-Device-JWT": "test"})
  assert sent[0][1]["json"]["fix_time"] == FIX_TIME + 5
  assert sent[0][1]["headers"] == {"X-Device-JWT": "test"}
  assert sent[0][0].endswith("/device-locations/device_1")
  assert "log_path" not in sent[0][1]["json"]
  assert sent[0][1]["json"]["route"] == "2026-10-14--12-00-00"
  assert Path(reporter.path).stat().st_mode & 0o777 == 0o600
  reporter.step("device_1", {})
  assert len(sent) == 1
  clock[0] += 61
  reporter.step("device_1", {})
  assert len(sent) == 2 and reads == [path]


def test_restart_and_deleted_logs_keep_last_fix(setup):
  reporter, params, _, sent, messages, _, logfile = setup
  path = logfile()
  messages[path] = [gps()]
  reporter.step("device_1", {})
  os.unlink(path)
  restored = location.LocationReporter(reporter.root, reporter.path, params=params, log_reader=lambda _: ())
  restored.step("device_1", {})
  assert sent[-1][1]["json"] == sent[0][1]["json"]
  assert restored.fix["fix_time"] == FIX_TIME


def test_new_log_is_detected_and_older_fix_cannot_replace_cache(setup):
  reporter, _, clock, sent, messages, _, logfile = setup
  first = logfile(age=100)
  messages[first] = [gps(20)]
  reporter.step("device_1", {})
  newer_file = logfile(1)
  messages[newer_file] = [gps(10)]
  clock[0] += 61
  reporter.step("device_1", {})
  assert sent[-1][1]["json"]["fix_time"] == FIX_TIME + 20
  Path(newer_file).write_bytes(b"updated")
  os.utime(newer_file, (location.time.time() - 60, location.time.time() - 60))
  messages[newer_file] = [gps(30)]
  clock[0] += 61
  reporter.step("device_1", {})
  assert sent[-1][1]["json"]["fix_time"] == FIX_TIME + 30


def test_qlog_preferred_and_rlog_is_used_when_qlog_has_no_fix(setup):
  reporter, _, _, sent, messages, reads, logfile = setup
  qlog, rlog = logfile(age=70), logfile(name="rlog", age=60)
  messages[qlog], messages[rlog] = [gps(has_fix=False)], [gps(10)]
  reporter.step("device_1", {})
  assert reads == [qlog, rlog]
  assert sent[-1][1]["json"]["fix_time"] == FIX_TIME + 10


def test_periodic_scan_is_bounded_and_param_fallback_has_unknown_age(setup):
  reporter, params, clock, sent, messages, reads, logfile = setup
  params.fallback = json.dumps({"latitude": 0, "longitude": 0})
  for index in range(5):
    path = logfile(index, age=60 + index)
    messages[path] = [gps(has_fix=False)] if index < 4 else [gps(20)]
  reporter.step("device_1", {})
  assert len(reads) == 3
  assert sent[-1][1]["json"] == {"latitude": 0, "longitude": 0, "source": "LastGPSPosition", "fix_time": None}
  clock[0] += 61
  reporter.step("device_1", {})
  assert len(reads) == 5
  assert sent[-1][1]["json"]["fix_time"] == FIX_TIME + 20


def test_off_switch_onroad_and_unsettled_logs_do_not_read(setup, monkeypatch):
  reporter, params, _, sent, messages, reads, logfile = setup
  path = logfile(age=2)
  messages[path] = [gps()]
  reporter.step("device_1", {})
  assert not sent and not reads
  reporter.last_poll = float("-inf")
  params.onroad = True
  reporter.step("device_1", {})
  assert not sent and not reads
  params.onroad = False
  monkeypatch.setenv("TELETYPED_LOCATION", "0")
  reporter.step("device_1", {})
  assert not sent and not reads


def test_no_valid_position_sends_nothing_and_cache_is_device_bound(setup):
  reporter, params, clock, sent, messages, _, logfile = setup
  reporter.step("device_1", {})
  assert not sent
  path = logfile()
  messages[path] = [gps()]
  clock[0] += 61
  reporter.step("device_1", {})
  os.unlink(path)
  restored = location.LocationReporter(reporter.root, reporter.path, params=params)
  restored.step("different_device", {})
  assert len(sent) == 1 and restored.fix is None


def test_log_network_failure_preserves_fix_for_retry(setup, monkeypatch):
  reporter, _, clock, sent, messages, _, logfile = setup
  path = logfile()
  messages[path] = [gps()]
  post = location.http_post
  monkeypatch.setattr(location, "http_post", lambda *args, **kwargs: (_ for _ in ()).throw(OSError("offline")))
  with pytest.raises(OSError):
    reporter.step("device_1", {})
  assert Path(reporter.path).exists()
  monkeypatch.setattr(location, "http_post", post)
  os.unlink(path)
  clock[0] += 61
  reporter.step("device_1", {})
  assert sent[-1][1]["json"]["fix_time"] == FIX_TIME


def test_real_capnp_gps_log_roundtrip(setup):
  from cereal import log
  from openpilot.tools.lib.logreader import LogReader

  reporter, _, _, sent, _, _, logfile = setup
  path = logfile()
  events = []
  for second, has_fix in [(1, True), (2, True), (3, False)]:
    event = log.Event.new_message(valid=True, logMonoTime=1000000000 * second)
    gps_message = event.init("gpsLocationExternal")
    gps_message.latitude, gps_message.longitude = 0, 24
    gps_message.hasFix = has_fix
    gps_message.unixTimestampMillis = (FIX_TIME + second) * 1000
    events.append(event.to_bytes())
  Path(path).write_bytes(b"".join(events))
  os.utime(path, (location.time.time() - 60, location.time.time() - 60))
  reporter.log_reader = LogReader
  reporter.step("device_1", {})
  assert sent[-1][1]["json"]["fix_time"] == FIX_TIME + 2
  assert sent[-1][1]["json"]["latitude"] == 0
