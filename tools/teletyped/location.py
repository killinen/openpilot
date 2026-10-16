"""Bounded, offroad reporting of the device's last known GPS fix."""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any

from openpilot.tools.teletyped.helper import API_URL, PERSIST_ROOT, REALDATA_DIR, http_post, log
from openpilot.tools.teletyped.label_utils import drive_base_name, is_drive_label

LOCATION_INTERVAL = 60
LOCATION_SETTLE_SECONDS = 30
LOCATION_SCAN_FILES_PER_TICK = 3
LOCATION_RETRY_SECONDS = 300
_reporter: LocationReporter | None = None


def _number(value: Any) -> float | None:
  if isinstance(value, bool):
    return None
  try:
    number = float(value)
    return number if math.isfinite(number) else None
  except (TypeError, ValueError):
    return None


def _coordinates(latitude: Any, longitude: Any) -> dict[str, float] | None:
  lat, lon = _number(latitude), _number(longitude)
  if lat is None or lon is None or not -90 <= lat <= 90 or not -180 <= lon <= 180:
    return None
  return {"latitude": lat, "longitude": lon}


def fix_from_message(message: Any) -> dict[str, Any] | None:
  kind = message.which()
  if kind not in {"gpsLocation", "gpsLocationExternal", "liveLocationKalman"} or not getattr(message, "valid", True):
    return None
  gps = getattr(message, kind)
  if kind == "liveLocationKalman":
    if not gps.gpsOK or not gps.positionGeodetic.valid or str(gps.status) != "valid":
      return None
    values = gps.positionGeodetic.value
    if len(values) < 2:
      return None
    coordinates = _coordinates(values[0], values[1])
  else:
    if not (getattr(gps, "hasFix", False) or int(getattr(gps, "flags", 0)) & 1):
      return None
    coordinates = _coordinates(gps.latitude, gps.longitude)
  timestamp = _number(gps.unixTimestampMillis)
  if coordinates is None or timestamp is None or not 946684800000 <= timestamp <= (time.time() + 300) * 1000:
    return None
  fix = {**coordinates, "fix_time": timestamp / 1000, "source": kind}
  if kind != "liveLocationKalman":
    for field, attribute in (("accuracy_m", "horizontalAccuracy"), ("speed_mps", "speed")):
      value = _number(getattr(gps, attribute, None))
      if value is not None and value >= 0:
        fix[field] = value
  return fix


def location_log_paths(root: str) -> list[str]:
  segments = []
  if not os.path.isdir(root):
    return []
  for entry in os.listdir(root):
    directory = os.path.join(root, entry)
    if not os.path.isdir(directory) or not is_drive_label(entry):
      continue
    # qlogs contain GPS and are much cheaper to read than full CAN logs.
    paths = []
    newest = 0
    for name in ("qlog", "qlog.bz2", "rlog", "rlog.bz2"):
      path = os.path.join(directory, name)
      try:
        stat = os.stat(path)
      except OSError:
        continue
      if os.path.isfile(path):
        paths.append(path)
        newest = max(newest, stat.st_mtime_ns)
    if paths:
      segments.append((newest, paths))
  return [path for _, paths in sorted(segments, reverse=True) for path in paths]


class LocationReporter:
  def __init__(self, root: str = REALDATA_DIR, state_path: str | None = None, *, params: Any = None, log_reader: Any = None):
    self.root = root
    self.path = state_path or os.path.join(PERSIST_ROOT, "comma", "teletyped_last_location.json")
    self.params = params
    self.log_reader = log_reader
    self.device_id: str | None = None
    self.fix: dict[str, Any] | None = None
    self.scanned: dict[str, tuple[int, int]] = {}
    self.failed: dict[str, float] = {}
    self.last_poll = float("-inf")

  def _params(self):
    if self.params is None:
      from openpilot.common.params import Params

      self.params = Params()
    return self.params

  def _restore(self, device_id: str):
    if self.device_id == device_id:
      return
    self.device_id, self.fix = device_id, None
    try:
      with open(self.path, "rb") as file:
        state = json.loads(file.read(8192))
      fix = state["fix"]
      if state.get("version") == 1 and state.get("device_id") == device_id and isinstance(fix, dict):
        coordinates = _coordinates(fix.get("latitude"), fix.get("longitude"))
        timestamp = _number(fix.get("fix_time"))
        if coordinates is not None and (fix.get("fix_time") is None or timestamp is not None and 946684800 <= timestamp <= time.time() + 300):
          self.fix = {**fix, **coordinates}
    except FileNotFoundError:
      pass
    except (ValueError, KeyError, TypeError):
      log("Last-location cache is invalid; preserving it and looking for a valid log fix", "WARN")

  def _save(self):
    os.makedirs(os.path.dirname(self.path), exist_ok=True)
    temporary = self.path + ".tmp"
    raw = json.dumps({"version": 1, "device_id": self.device_id, "fix": self.fix}, allow_nan=False).encode()
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(descriptor, 0o600)
    with os.fdopen(descriptor, "wb") as file:
      file.write(raw)
      file.flush()
      os.fsync(file.fileno())
    os.replace(temporary, self.path)

  def _scan(self, now: float):
    paths = location_log_paths(self.root)
    existing = set(paths)
    self.scanned = {key: value for key, value in self.scanned.items() if key in existing}
    self.failed = {key: value for key, value in self.failed.items() if key in existing}
    read = 0
    for path in paths:
      if self._params().get_bool("IsOnroad"):
        return
      stat = os.stat(path)
      signature = (stat.st_size, stat.st_mtime_ns)
      if time.time() - stat.st_mtime < LOCATION_SETTLE_SECONDS:
        # Wait for the newest file; reporting an older fix is still safe.
        continue
      if self.scanned.get(path) == signature:
        # A known valid newest segment already gave us its last fix.
        if self.fix and self.fix.get("log_path") == path:
          return
        continue
      if now - self.failed.get(path, float("-inf")) < LOCATION_RETRY_SECONDS:
        continue
      if read >= LOCATION_SCAN_FILES_PER_TICK:
        return
      read += 1
      if self.log_reader is None:
        from openpilot.tools.lib.logreader import LogReader

        self.log_reader = LogReader
      latest: dict[str, Any] | None = None
      try:
        for index, message in enumerate(self.log_reader(path)):
          if index % 1000 == 0 and self._params().get_bool("IsOnroad"):
            return
          candidate = fix_from_message(message)
          if candidate and (latest is None or candidate["fix_time"] > latest["fix_time"]):
            latest = candidate
        current = os.stat(path)
        if (current.st_size, current.st_mtime_ns) != signature:
          return
      except Exception:
        self.failed[path] = now
        log("Could not read a location log; retaining the last known fix", "WARN")
        continue
      self.scanned[path] = signature
      if latest:
        if self.fix is None or self.fix.get("fix_time") is None or latest["fix_time"] >= self.fix["fix_time"]:
          directory = os.path.basename(os.path.dirname(path))
          segment = directory.rsplit("--", 1)[-1]
          self.fix = {**latest, "route": drive_base_name(directory), "segment": int(segment) if segment.isdigit() else None, "log_path": path}
          self._save()
        return

  def _param_fallback(self):
    # This persistent parameter has coordinates but no capture timestamp. Never
    # label its read time as GPS time or overwrite a timestamped cached fix.
    if self.fix is None:
      try:
        value = json.loads(self._params().get("LastGPSPosition") or "null")
        coordinates = _coordinates(value.get("latitude"), value.get("longitude")) if isinstance(value, dict) else None
        if coordinates:
          self.fix = {**coordinates, "source": "LastGPSPosition", "fix_time": None}
          self._save()
      except (TypeError, ValueError):
        pass

  def step(self, device_id: str, headers: dict[str, str]) -> None:
    if os.getenv("TELETYPED_LOCATION", "1").strip().lower() in {"0", "false", "no", "off"} or self._params().get_bool("IsOnroad"):
      return
    now = time.monotonic()
    if now - self.last_poll < LOCATION_INTERVAL:
      return
    self.last_poll = now
    self._restore(device_id)
    self._param_fallback()
    self._scan(now)
    if self.fix is not None and not self._params().get_bool("IsOnroad"):
      payload = {key: value for key, value in self.fix.items() if key != "log_path"}
      response = http_post(f"{API_URL}/device-locations/{device_id}", headers=headers, json=payload, timeout=10)
      response.raise_for_status()


def location_step(device_id: str, headers: dict[str, str]) -> None:
  global _reporter
  if _reporter is None:
    _reporter = LocationReporter()
  _reporter.step(device_id, headers)
