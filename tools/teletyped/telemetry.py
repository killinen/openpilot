"""Offroad telemetry dispatch, durable retries, and transport for optional collectors."""

from __future__ import annotations

from collections.abc import Callable
from datetime import UTC, datetime
import gzip
import json
import os
import time
from typing import Any

from openpilot.tools.teletyped.collectors import Collector, enabled_collectors
from openpilot.tools.teletyped.helper import API_URL, PERSIST_ROOT, capture_exception, http_post, log
from openpilot.tools.teletyped.route_logs import DriveLogSet, _segment_number, discover_drive_logs


TELEMETRY_UPLOAD_PATH = f"{API_URL}/drive-telemetry"


def _env_int(name: str, default: int, minimum: int) -> int:
  try:
    return max(minimum, int(os.environ.get(name, str(default))))
  except ValueError:
    return default


TELEMETRY_MAX_PER_TICK = _env_int("TELETYPED_TELEMETRY_MAX_PER_TICK", 1, 1)
TELEMETRY_RETRY_INTERVAL = _env_int("TELETYPED_TELEMETRY_RETRY_INTERVAL", 900, 0)
TELEMETRY_SETTLE_SECONDS = 120
TELEMETRY_BATCH_SIZE = 600
_TELEMETRY_STATE_PATH: str | None = None


class AnalysisInterrupted(Exception):
  pass


def _is_onroad() -> bool:
  from openpilot.common.params import Params

  return bool(Params().get_bool("IsOnroad"))


def analyze_telemetry(
  drive: DriveLogSet, device_id: str, collectors: list[Collector],
  log_reader: Any | None = None, should_stop: Callable[[], bool] = _is_onroad,
) -> list[dict[str, Any]]:
  if log_reader is None:
    from openpilot.tools.lib.logreader import LogReader

    log_reader = LogReader

  if not drive.paths or should_stop():
    raise AnalysisInterrupted

  def read_messages(path: str) -> list[Any]:
    if should_stop():
      raise AnalysisInterrupted
    return list(log_reader(path, sort_by_time=True))

  # CAN can precede carParams by several seconds. Resolve logged identity before
  # replaying samples, retaining only one segment at a time in memory.
  first_messages = read_messages(drive.paths[0])
  vehicle: tuple[str, str] | None = None
  for index, path in enumerate(drive.paths):
    messages = first_messages if index == 0 else read_messages(path)
    cp_message = next((msg for msg in messages if msg.which() == "carParams"), None)
    if cp_message is not None:
      vehicle = (str(cp_message.carParams.carName), str(cp_message.carParams.carFingerprint))
      break
  if vehicle is None:
    return []
  active = [collector for collector in collectors if collector.matches(*vehicle)]
  if not active:
    return []
  start_ns: int | None = None
  recorded_at: str | None = None
  metadata: dict[str, Any] = {}
  for path_index, path in enumerate(drive.paths):
    if should_stop():
      raise AnalysisInterrupted
    messages = first_messages if path_index == 0 else read_messages(path)
    segment = _segment_number(path)
    for index, msg in enumerate(messages):
      if index % 1000 == 0 and should_stop():
        raise AnalysisInterrupted
      now_ns = int(msg.logMonoTime)
      if start_ns is None:
        start_ns = now_ns
      which = msg.which()
      if which == "carParams":
        logged_vehicle = (str(msg.carParams.carName), str(msg.carParams.carFingerprint))
        if vehicle != logged_vehicle:
          raise ValueError("Vehicle metadata changed within the route")
      elif which == "initData":
        init = msg.initData
        wall_ns = int(getattr(init, "wallTimeNanos", 0))
        if wall_ns > 0 and recorded_at is None:
          recorded_at = datetime.fromtimestamp((wall_ns + start_ns - now_ns) / 1e9, UTC).isoformat()
        metadata = {
          "version": str(getattr(init, "version", "")) or None,
          "git_commit": str(getattr(init, "gitCommit", "")) or None,
          "git_branch": str(getattr(init, "gitBranch", "")) or None,
        }
      for collector in active:
        collector.observe(msg, (now_ns - start_ns) / 1e9, segment)
    if path_index == 0:
      first_messages = []

  payloads = []
  for collector in active:
    result = collector.result()
    if result is None:
      continue
    samples = result.pop("samples")
    first_batch = result.pop("first_batch", {})
    batch_count = max(1, (len(samples) + TELEMETRY_BATCH_SIZE - 1) // TELEMETRY_BATCH_SIZE)
    for batch_index in range(batch_count):
      payloads.append({
        "schema_version": collector.schema_version,
        "collector": collector.key,
        "device_id": device_id,
        "drive": drive.name,
        "source_fingerprint": drive.fingerprint,
        "car_name": vehicle[0] if vehicle else "",
        "car_fingerprint": vehicle[1] if vehicle else "",
        "recorded_at": recorded_at,
        "start_mono_time_ns": start_ns,
        "segment_count": len(drive.paths),
        "segments": [_segment_number(path) for path in drive.paths],
        "batch_index": batch_index,
        "batch_count": batch_count,
        "generated_at": datetime.now(UTC).isoformat(),
        **metadata,
        "data": {
          **result, **(first_batch if batch_index == 0 else {}),
          "samples": samples[batch_index * TELEMETRY_BATCH_SIZE:(batch_index + 1) * TELEMETRY_BATCH_SIZE],
        },
      })
  return payloads


def _state_path() -> str:
  global _TELEMETRY_STATE_PATH
  if _TELEMETRY_STATE_PATH is None:
    state_dir = os.environ.get("TELETYPED_STATE_DIR")
    directories = (state_dir,) if state_dir else (os.path.join(PERSIST_ROOT, "comma"), os.path.join(PERSIST_ROOT, "teletyped"), "/tmp/comma")
    for directory in directories:
      path = os.path.join(directory, "teletyped_telemetry_state.json.gz")
      if os.path.isfile(path) or os.path.isfile(path[:-3]):
        _TELEMETRY_STATE_PATH = path
        return path
    for directory in directories:
      try:
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, "teletyped_telemetry_state.json.gz")
        with open(path + ".write-test", "w", encoding="utf-8"):
          pass
        os.remove(path + ".write-test")
        _TELEMETRY_STATE_PATH = path
        break
      except OSError:
        continue
    if _TELEMETRY_STATE_PATH is None:
      raise OSError("No writable telemetry state directory")
  return _TELEMETRY_STATE_PATH


def _load_state() -> dict[str, Any]:
  try:
    path = _state_path()
    if not os.path.exists(path) and path.endswith(".gz"):
      path = path[:-3]  # Migrate a pre-compression queue without losing retries.
    with open(path, "rb") as file:
      raw = file.read()
    state = json.loads(gzip.decompress(raw) if raw.startswith(b"\x1f\x8b") else raw)
    if isinstance(state, dict) and state.get("version") == 1 and isinstance(state.get("jobs"), dict):
      return state
    raise ValueError("Invalid telemetry queue; preserving it for recovery")
  except FileNotFoundError:
    return {"version": 1, "jobs": {}}


def _save_state(state: dict[str, Any]) -> None:
  path = _state_path()
  raw = json.dumps(state, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
  with open(path + ".tmp", "wb") as file:
    file.write(gzip.compress(raw, compresslevel=6, mtime=0))
    file.flush()
    os.fsync(file.fileno())
  os.replace(path + ".tmp", path)
  if path.endswith(".gz") and os.path.isfile(path[:-3]):
    os.remove(path[:-3])


def _job_key(drive_name: str, collector: Collector) -> str:
  return f"{collector.key}:{collector.schema_version}:{drive_name}"


def _needs_analysis(job: dict[str, Any], fingerprint: str) -> bool:
  if job.get("status") == "pending_upload":
    return False
  if job.get("fingerprint") != fingerprint:
    return True
  if job.get("status") in {"uploaded", "no_data"}:
    return False
  elapsed = time.time() - job.get("last_attempt_at", 0)
  return not 0 <= elapsed < TELEMETRY_RETRY_INTERVAL


def _upload_pending(state: dict[str, Any], key: str, headers: dict[str, str]) -> None:
  job = state["jobs"][key]
  try:
    while job["next_batch"] < len(job["payloads"]):
      if _is_onroad():
        return
      job["last_attempt_at"] = time.time()
      _save_state(state)
      payload = job["payloads"][job["next_batch"]]
      raw = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
      response = http_post(
        TELEMETRY_UPLOAD_PATH, data=gzip.compress(raw, compresslevel=6, mtime=0),
        headers={**headers, "Content-Type": "application/json", "Content-Encoding": "gzip"}, timeout=30,
      )
      # This endpoint only returns 409 for an identical stored batch.
      if response.status_code != 409:
        response.raise_for_status()
      job["next_batch"] += 1
      _save_state(state)
    state["jobs"][key] = {"fingerprint": job["fingerprint"], "status": "uploaded"}
    _save_state(state)
    log(f"Uploaded vehicle telemetry: {key}")
  except Exception as error:
    job["last_error"] = str(error)[:300]
    _save_state(state)
    capture_exception(error)
    log(f"Vehicle telemetry upload failed for {key}: {error}", "WARN")


def telemetry_step(device_id: str, headers: dict[str, str]) -> None:
  collectors = enabled_collectors()
  if not collectors or _is_onroad():
    return
  state = _load_state()
  processed = 0
  enabled_keys = {collector.key for collector in collectors}
  # Retry durable payloads even if loggerd has since deleted their source files.
  for key, job in list(state["jobs"].items()):
    if key.split(":", 1)[0] not in enabled_keys or job.get("status") != "pending_upload":
      continue
    elapsed = time.time() - job.get("last_attempt_at", 0)
    if 0 <= elapsed < TELEMETRY_RETRY_INTERVAL:
      continue
    _upload_pending(state, key, headers)
    processed += 1
    if processed >= TELEMETRY_MAX_PER_TICK or _is_onroad():
      return

  for drive in discover_drive_logs(rlogs_only=True):
    if _is_onroad():
      return
    missing = [collector for collector in collectors if _needs_analysis(
      state["jobs"].get(_job_key(drive.name, collector), {}), drive.fingerprint,
    )]
    if not missing:
      continue
    try:
      if any(time.time() - os.stat(path).st_mtime < TELEMETRY_SETTLE_SECONDS for path in drive.paths):
        continue
      # Collectors hold cross-segment freshness/counters for this route only.
      fresh = [type(collector)() for collector in missing]
      payloads = analyze_telemetry(drive, device_id, fresh)
      # Do not queue a snapshot that changed while being read.
      current = next((item for item in discover_drive_logs(rlogs_only=True) if item.name == drive.name), None)
      if current is None or current.fingerprint != drive.fingerprint:
        continue
      for collector in missing:
        key = _job_key(drive.name, collector)
        batches = [payload for payload in payloads if payload["collector"] == collector.key]
        state["jobs"][key] = {
          "fingerprint": drive.fingerprint,
          "status": "pending_upload" if batches else "no_data",
          **({"payloads": batches, "next_batch": 0} if batches else {}),
        }
      _save_state(state)
      for collector in missing:
        key = _job_key(drive.name, collector)
        if state["jobs"][key]["status"] == "pending_upload":
          _upload_pending(state, key, headers)
    except AnalysisInterrupted:
      return
    except Exception as error:
      capture_exception(error)
      # Retry analysis later rather than permanently losing a transient read failure.
      for collector in missing:
        state["jobs"][_job_key(drive.name, collector)] = {
          "fingerprint": drive.fingerprint, "status": "analysis_error",
          "last_attempt_at": time.time(), "last_error": str(error)[:300],
        }
      _save_state(state)
      log(f"Vehicle telemetry analysis failed for {drive.name}: {error}", "WARN")
    processed += 1
    if processed >= TELEMETRY_MAX_PER_TICK:
      return
