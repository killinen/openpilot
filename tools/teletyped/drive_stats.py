#!/usr/bin/env python3
"""Generate per-drive engagement statistics while the device is offroad."""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass
from datetime import UTC, datetime
import hashlib
import json
import os
import time
from typing import Any

from openpilot.tools.teletyped.helper import API_URL, PERSIST_ROOT, REALDATA_DIR, capture_exception, http_post, log
from openpilot.tools.teletyped.label_utils import drive_base_name, is_drive_label


def _env_positive_int(name: str, default: int) -> int:
  try:
    return max(1, int(os.environ.get(name, str(default))))
  except ValueError:
    return default


def _env_nonnegative_int(name: str, default: int) -> int:
  try:
    return max(0, int(os.environ.get(name, str(default))))
  except ValueError:
    return default


DRIVE_STATS_UPLOAD_PATH = f"{API_URL}/drive-stats"
DRIVE_STATS_STATE_FILE = "teletyped_drive_stats_state.json"
DRIVE_STATS_SCHEMA_VERSION = 3
DRIVE_STATS_MAX_PER_TICK = _env_positive_int("TELETYPED_DRIVE_STATS_MAX_PER_TICK", 1)
DRIVE_STATS_RETRY_INTERVAL = _env_nonnegative_int("TELETYPED_DRIVE_STATS_RETRY_INTERVAL", 900)
DRIVE_STATS_ENABLED = os.environ.get("TELETYPED_DRIVE_STATS", "1").strip().lower() not in {
  "0",
  "false",
  "no",
  "off",
}

STEER_INTERVENTION_THRESHOLD = 1.5
STEER_RESOLUTION_THRESHOLD = 0.3
INTERVENTION_TIMEOUT_NS = 10 * 1_000_000_000
ENGAGEMENT_BUFFER_NS = 3 * 1_000_000_000
MAX_SAMPLE_GAP_NS = 1 * 1_000_000_000

SPEED_BUCKETS: tuple[dict[str, Any], ...] = (
  {"key": "city", "label": "City (<55 km/h)", "min_speed_mps": 0.0, "max_speed_mps": 15.3},
  {"key": "road", "label": "Road (55-90 km/h)", "min_speed_mps": 15.3, "max_speed_mps": 25.0},
  {"key": "highway", "label": "Highway (>=90 km/h)", "min_speed_mps": 25.0, "max_speed_mps": None},
)

_DRIVE_STATS_STATE_PATH: str | None = None


def _speed_bucket_for(speed_mps: float) -> str:
  speed = max(0.0, speed_mps)
  for bucket in SPEED_BUCKETS:
    maximum = bucket["max_speed_mps"]
    if speed >= bucket["min_speed_mps"] and (maximum is None or speed < maximum):
      return str(bucket["key"])
  return "highway"


def _new_speed_bucket_state() -> dict[str, dict[str, int | float]]:
  return {
    str(bucket["key"]): {
      "time_ns": 0,
      "engaged_time_ns": 0,
      "drive_time_ns": 0,
      "engaged_drive_time_ns": 0,
      "distance_km": 0.0,
      "engaged_distance_km": 0.0,
      "raw_disengagement_count": 0,
      "torque_intervention_count": 0,
      "pressed_intervention_count": 0,
    }
    for bucket in SPEED_BUCKETS
  }


class MovingAverage:
  def __init__(self, window: int = 10) -> None:
    self.values: deque[float] = deque(maxlen=window)

  def update(self, value: float) -> float:
    self.values.append(float(value))
    return sum(self.values) / len(self.values)


class VehicleStatsProfile:
  """Vehicle-specific decoding boundary for adding future cars."""

  name = "unknown"

  def matches(self, car_name: str, fingerprint: str) -> bool:
    raise NotImplementedError

  def decode_odometer_km(self, source: int, address: int, data: bytes) -> float | None:
    return None

  def has_legacy_steering_feedback(self, source: int, address: int) -> bool:
    return False

  def steering_values(self, car_state: Any) -> tuple[float, float]:
    raise NotImplementedError


class HyundaiI30StatsProfile(VehicleStatsProfile):
  name = "hyundai_i30_2014"
  fingerprints = {
    "HYUNDAI I30 GD 2014",
    "HYUNDAI_I30_GD_2014",
  }

  def matches(self, car_name: str, fingerprint: str) -> bool:
    return car_name.strip().lower() == "i30" or fingerprint.strip().upper() in self.fingerprints

  def decode_odometer_km(self, source: int, address: int, data: bytes) -> float | None:
    # CLU1.CF_Clu_Odometer: Intel 24-bit value at bit 40, scale 0.1 km.
    if source != 0 or address != 1264 or len(data) < 8:
      return None
    return ((data[7] << 16) | (data[6] << 8) | data[5]) * 0.1

  def has_legacy_steering_feedback(self, source: int, address: int) -> bool:
    # The legacy SSC actuator publishes STEERING_STATUS on camera bus 1.
    return source == 1 and address == 559

  def steering_values(self, car_state: Any) -> tuple[float, float]:
    # Preserve the calibration/sign convention used by opDriveStats.
    driver = -float(getattr(car_state, "steeringTorque", 0.0)) - 0.2
    actuator = float(getattr(car_state, "steeringTorqueEps", 0.0))
    return driver, actuator


VEHICLE_PROFILES: tuple[VehicleStatsProfile, ...] = (HyundaiI30StatsProfile(),)


def select_vehicle_profile(car_name: str = "", fingerprint: str = "") -> VehicleStatsProfile | None:
  for profile in VEHICLE_PROFILES:
    if profile.matches(car_name, fingerprint):
      return profile
  return None


@dataclass(frozen=True)
class DriveLogSet:
  name: str
  paths: tuple[str, ...]
  fingerprint: str


def _segment_number(path: str) -> int:
  name = os.path.basename(os.path.dirname(path))
  suffix = name.rsplit("--", 1)[-1]
  return int(suffix) if suffix.isdigit() else -1


def _preferred_log_path(segment_path: str) -> str | None:
  for basename in ("rlog", "rlog.bz2", "qlog", "qlog.bz2"):
    candidate = os.path.join(segment_path, basename)
    if os.path.isfile(candidate):
      return candidate
  return None


def discover_drive_logs(realdata_dir: str = REALDATA_DIR) -> list[DriveLogSet]:
  grouped: dict[str, list[str]] = {}
  if not os.path.isdir(realdata_dir):
    return []

  for entry in sorted(os.listdir(realdata_dir)):
    entry_path = os.path.join(realdata_dir, entry)
    if entry == "boot" or not os.path.isdir(entry_path) or not is_drive_label(entry):
      continue
    log_path = _preferred_log_path(entry_path)
    if log_path is not None:
      grouped.setdefault(drive_base_name(entry), []).append(log_path)

  result: list[DriveLogSet] = []
  for name, paths in grouped.items():
    paths.sort(key=lambda path: (_segment_number(path), path))
    source_parts: list[str] = []
    for path in paths:
      try:
        stat = os.stat(path)
      except OSError:
        continue
      source_parts.append(f"{os.path.basename(os.path.dirname(path))}:{os.path.basename(path)}:{stat.st_size}:{stat.st_mtime_ns}")
    if not source_parts:
      continue
    digest = hashlib.sha256("\n".join(source_parts).encode("utf-8")).hexdigest()
    result.append(DriveLogSet(name=name, paths=tuple(paths), fingerprint=digest))

  # Process fresh logs first; old retained routes are backfilled one per tick.
  return sorted(result, key=lambda drive: drive.name, reverse=True)


def _current_vehicle_profile() -> VehicleStatsProfile | None:
  try:
    from cereal import car
    from openpilot.common.params import Params

    params = Params()
    raw = params.get("CarParamsPersistent") or params.get("CarParams") or params.get("CarParamsCache")
    if raw:
      with car.CarParams.from_bytes(raw) as cp:
        return select_vehicle_profile(str(cp.carName), str(cp.carFingerprint))
  except Exception as e:
    capture_exception(e)
  return None


class DriveAnalyzer:
  def __init__(self, drive_name: str, device_id: str, profile: VehicleStatsProfile | None = None) -> None:
    self.drive_name = drive_name
    self.device_id = device_id
    self.profile = profile
    self.car_name = ""
    self.car_fingerprint = ""
    self.version: str | None = None
    self.git_branch: str | None = None
    self.git_commit: str | None = None
    self.git_commit_date: str | None = None
    self.git_dirty: bool | None = None
    self.device_type: str | None = None
    self.recorded_at: str | None = None
    self.openpilot_longitudinal: bool | None = None
    self.segment_count = 0

    self.total_time_ns = 0
    self.engaged_time_ns = 0
    self.drive_time_ns = 0
    self.engaged_drive_time_ns = 0
    self.total_distance_km = 0.0
    self.engaged_distance_km = 0.0
    self.integrated_distance_km = 0.0
    self.integrated_engaged_distance_km = 0.0
    self.odometer_samples = 0
    self.last_odometer_km: float | None = None

    self.engaged = False
    self.last_engagement_state: bool | None = None
    self.last_engagement_change_ns: int | None = None
    self.engagement_state_changes = 0
    self.disengagement_count = 0
    self.final_disengagement_pending = False
    self.final_disengagement_bucket: str | None = None
    self.current_speed_mps = 0.0
    self.speed_bucket_state = _new_speed_bucket_state()

    self.driver_filter = MovingAverage()
    self.actuator_filter = MovingAverage()
    self.in_torque_intervention = False
    self.last_torque_intervention_ns = 0
    self.torque_intervention_count = 0
    self.in_pressed_intervention = False
    self.last_pressed_intervention_ns = 0
    self.pressed_intervention_count = 0
    self.saw_legacy_steering_feedback = False
    self.trqi_steering: bool | None = None

  def _set_vehicle(self, car_params: Any) -> None:
    self.car_name = str(getattr(car_params, "carName", ""))
    self.car_fingerprint = str(getattr(car_params, "carFingerprint", ""))
    self.openpilot_longitudinal = bool(getattr(car_params, "openpilotLongitudinalControl", False))
    # Logged metadata is authoritative. This prevents the current car's cached
    # Params from being applied to an old route recorded by another vehicle.
    self.profile = select_vehicle_profile(self.car_name, self.car_fingerprint)

  def _observe_init_data(self, init_data: Any) -> None:
    self.version = str(getattr(init_data, "version", "")) or None
    self.git_branch = str(getattr(init_data, "gitBranch", "")) or None
    self.git_commit = str(getattr(init_data, "gitCommit", "")) or None
    self.git_commit_date = str(getattr(init_data, "gitCommitDate", "")) or None
    self.git_dirty = bool(getattr(init_data, "dirty", False))
    device_type = getattr(init_data, "deviceType", None)
    self.device_type = str(device_type).split(".")[-1] if device_type is not None else None
    wall_time_ns = int(getattr(init_data, "wallTimeNanos", 0))
    if wall_time_ns > 0 and self.recorded_at is None:
      self.recorded_at = datetime.fromtimestamp(wall_time_ns / 1e9, UTC).isoformat()
    entries = getattr(getattr(init_data, "params", None), "entries", ())
    for entry in entries:
      if str(getattr(entry, "key", "")) != "TRQISteeringToggle":
        continue
      value = bytes(getattr(entry, "value", b"")).strip().lower()
      self.trqi_steering = value in {b"1", b"true", b"yes", b"on"}
      break

  def _engagement_stable(self, now_ns: int) -> bool:
    return self.last_engagement_change_ns is None or now_ns - self.last_engagement_change_ns > ENGAGEMENT_BUFFER_NS

  def _observe_torque(self, now_ns: int, car_state: Any) -> None:
    if self.profile is None:
      return
    driver, actuator = self.profile.steering_values(car_state)
    filtered_driver = self.driver_filter.update(driver)
    filtered_actuator = self.actuator_filter.update(actuator)
    difference = abs(filtered_driver - filtered_actuator)
    if not self.engaged or not self._engagement_stable(now_ns):
      return
    if difference > STEER_INTERVENTION_THRESHOLD and not self.in_torque_intervention and now_ns - self.last_torque_intervention_ns > INTERVENTION_TIMEOUT_NS:
      self.torque_intervention_count += 1
      self.speed_bucket_state[_speed_bucket_for(self.current_speed_mps)]["torque_intervention_count"] += 1
      self.in_torque_intervention = True
      self.last_torque_intervention_ns = now_ns
    elif difference < STEER_RESOLUTION_THRESHOLD and self.in_torque_intervention:
      self.in_torque_intervention = False

  def _observe_pressed(self, now_ns: int, car_state: Any) -> None:
    pressed = bool(getattr(car_state, "steeringPressed", False))
    if (
      pressed
      and not self.in_pressed_intervention
      and self.engaged
      and self._engagement_stable(now_ns)
      and now_ns - self.last_pressed_intervention_ns > INTERVENTION_TIMEOUT_NS
    ):
      self.pressed_intervention_count += 1
      self.speed_bucket_state[_speed_bucket_for(self.current_speed_mps)]["pressed_intervention_count"] += 1
      self.last_pressed_intervention_ns = now_ns
    self.in_pressed_intervention = pressed

  def process_segment(self, messages: Iterable[Any]) -> None:
    self.segment_count += 1
    first_controls_ns: int | None = None
    last_controls_ns: int | None = None
    previous_controls_active = False
    last_car_state_ns: int | None = None
    previous_speed_mps = 0.0
    previous_car_state_engaged = self.engaged

    # Filters and intervention latches intentionally reset at segment boundaries,
    # matching the original opDriveStats behavior.
    self.driver_filter = MovingAverage()
    self.actuator_filter = MovingAverage()
    self.in_torque_intervention = False
    self.in_pressed_intervention = False

    for msg in messages:
      which = msg.which()
      now_ns = int(msg.logMonoTime)
      if which == "initData":
        self._observe_init_data(msg.initData)
      elif which == "carParams":
        self._set_vehicle(msg.carParams)
      elif which == "controlsState":
        active = bool(msg.controlsState.active)
        if first_controls_ns is None:
          first_controls_ns = now_ns
        if last_controls_ns is not None:
          delta = now_ns - last_controls_ns
          if 0 < delta <= MAX_SAMPLE_GAP_NS and previous_controls_active:
            self.engaged_time_ns += delta
        if self.last_engagement_state is not None and active != self.last_engagement_state:
          self.engagement_state_changes += 1
          self.last_engagement_change_ns = now_ns
          if self.last_engagement_state and not active:
            self.disengagement_count += 1
            self.final_disengagement_pending = True
            self.final_disengagement_bucket = _speed_bucket_for(self.current_speed_mps)
            self.speed_bucket_state[self.final_disengagement_bucket]["raw_disengagement_count"] += 1
          elif active:
            # A re-engagement proves the preceding disengagement was part of
            # the drive rather than the final shutdown transition.
            self.final_disengagement_pending = False
            self.final_disengagement_bucket = None
        self.last_engagement_state = active
        self.engaged = active
        previous_controls_active = active
        last_controls_ns = now_ns
      elif which == "can" and self.profile is not None:
        for can_msg in msg.can:
          source = int(can_msg.src)
          address = int(can_msg.address)
          data = bytes(can_msg.dat)
          if self.profile.has_legacy_steering_feedback(source, address):
            self.saw_legacy_steering_feedback = True
          odometer_km = self.profile.decode_odometer_km(source, address, data)
          if odometer_km is None:
            continue
          self.odometer_samples += 1
          if self.last_odometer_km is not None:
            delta_km = odometer_km - self.last_odometer_km
            if 0 < delta_km < 1.0:
              self.total_distance_km += delta_km
              if self.engaged:
                self.engaged_distance_km += delta_km
          self.last_odometer_km = odometer_km
      elif which == "carState":
        car_state = msg.carState
        current_speed_mps = float(getattr(car_state, "vEgo", 0.0))
        if last_car_state_ns is not None:
          delta_ns = now_ns - last_car_state_ns
          if 0 < delta_ns <= MAX_SAMPLE_GAP_NS:
            delta_km = max(0.0, previous_speed_mps) * delta_ns / 1e9 / 1000.0
            bucket_state = self.speed_bucket_state[_speed_bucket_for(previous_speed_mps)]
            bucket_state["time_ns"] += delta_ns
            bucket_state["distance_km"] += delta_km
            self.integrated_distance_km += delta_km
            if previous_car_state_engaged:
              bucket_state["engaged_time_ns"] += delta_ns
              bucket_state["engaged_distance_km"] += delta_km
              self.integrated_engaged_distance_km += delta_km
            if previous_speed_mps > 1.0:
              bucket_state["drive_time_ns"] += delta_ns
              self.drive_time_ns += delta_ns
              if previous_car_state_engaged:
                bucket_state["engaged_drive_time_ns"] += delta_ns
                self.engaged_drive_time_ns += delta_ns
        self.current_speed_mps = current_speed_mps
        self._observe_torque(now_ns, car_state)
        self._observe_pressed(now_ns, car_state)
        previous_speed_mps = current_speed_mps
        previous_car_state_engaged = self.engaged
        last_car_state_ns = now_ns

    if first_controls_ns is not None and last_controls_ns is not None:
      self.total_time_ns += max(0, last_controls_ns - first_controls_ns)

  def result(self) -> dict[str, Any] | None:
    if self.profile is None:
      return None

    use_odometer = self.odometer_samples >= 2
    total_distance = self.total_distance_km if use_odometer else self.integrated_distance_km
    engaged_distance = self.engaged_distance_km if use_odometer else self.integrated_engaged_distance_km
    uses_pressed_interventions = self.trqi_steering is True or (self.trqi_steering is None and not self.saw_legacy_steering_feedback)
    steering_interventions = self.pressed_intervention_count if uses_pressed_interventions else self.torque_intervention_count
    engagement_pct = self.engaged_time_ns / self.total_time_ns * 100 if self.total_time_ns else 0.0
    distance_pct = engaged_distance / total_distance * 100 if total_distance > 0 else None
    steer_per_100km = steering_interventions / total_distance * 100 if total_distance > 0 else None
    shutdown_disengagements_removed = int(self.final_disengagement_pending and self.disengagement_count > 0)
    corrected_disengagement_count = self.disengagement_count - shutdown_disengagements_removed
    disengagements_per_100km = corrected_disengagement_count / total_distance * 100 if total_distance > 0 else None
    disengagements_per_drive_hour = corrected_disengagement_count / (self.drive_time_ns / 3.6e12) if self.drive_time_ns > 0 else None
    speed_buckets: dict[str, dict[str, Any]] = {}
    for definition in SPEED_BUCKETS:
      key = str(definition["key"])
      bucket = self.speed_bucket_state[key]
      distance_km = float(bucket["distance_km"])
      engaged_distance_km = float(bucket["engaged_distance_km"])
      time_ns = int(bucket["time_ns"])
      engaged_time_ns = int(bucket["engaged_time_ns"])
      drive_time_ns = int(bucket["drive_time_ns"])
      engaged_drive_time_ns = int(bucket["engaged_drive_time_ns"])
      raw_disengagement_count = int(bucket["raw_disengagement_count"])
      shutdown_removed = int(shutdown_disengagements_removed > 0 and self.final_disengagement_bucket == key)
      bucket_disengagement_count = raw_disengagement_count - shutdown_removed
      bucket_steering_interventions = int(
        bucket["pressed_intervention_count"] if uses_pressed_interventions else bucket["torque_intervention_count"]
      )
      speed_buckets[key] = {
        "label": definition["label"],
        "min_speed_mps": definition["min_speed_mps"],
        "max_speed_mps": definition["max_speed_mps"],
        "time_ns": time_ns,
        "engaged_time_ns": engaged_time_ns,
        "drive_time_ns": drive_time_ns,
        "engaged_drive_time_ns": engaged_drive_time_ns,
        "time_min": round(time_ns / 60e9, 2),
        "engaged_time_min": round(engaged_time_ns / 60e9, 2),
        "distance_km_raw": round(distance_km, 6),
        "engaged_distance_km_raw": round(engaged_distance_km, 6),
        "distance_km": round(distance_km, 3),
        "engaged_distance_km": round(engaged_distance_km, 3),
        "engagement_pct": round(engaged_time_ns / time_ns * 100, 2) if time_ns else 0.0,
        "drive_time_engagement_pct": round(engaged_drive_time_ns / drive_time_ns * 100, 2) if drive_time_ns else 0.0,
        "distance_engagement_pct": round(engaged_distance_km / distance_km * 100, 2) if distance_km > 0 else None,
        "steer_intervention_count": bucket_steering_interventions,
        "steer_interventions_per_100km": round(bucket_steering_interventions / distance_km * 100, 2) if distance_km > 0 else None,
        "raw_disengagement_count": raw_disengagement_count,
        "disengagement_count": bucket_disengagement_count,
        "disengagements_per_100km": round(bucket_disengagement_count / distance_km * 100, 2) if distance_km > 0 else None,
        "disengagements_per_drive_hour": round(bucket_disengagement_count / (drive_time_ns / 3.6e12), 2) if drive_time_ns > 0 else None,
        "manual_shutdown_removed": shutdown_removed,
      }

    return {
      "schema_version": DRIVE_STATS_SCHEMA_VERSION,
      "device_id": self.device_id,
      "host": "device",
      "drive": self.drive_name,
      "recorded_at": self.recorded_at,
      "segment_count": self.segment_count,
      "vehicle_profile": self.profile.name,
      "car_name": self.car_name or None,
      "car_fingerprint": self.car_fingerprint or None,
      "total_time": self.total_time_ns,
      "active_time": self.engaged_time_ns,
      "drive_time": self.drive_time_ns,
      "drive_time_active": self.engaged_drive_time_ns,
      "engagement_pct": round(engagement_pct, 2),
      "drive_time_engagement_pct": round(self.engaged_drive_time_ns / self.drive_time_ns * 100, 2) if self.drive_time_ns else 0.0,
      "odo_distance": round(total_distance, 3),
      "engaged_distance": round(engaged_distance, 3),
      "engagement_pct_odo": round(distance_pct, 2) if distance_pct is not None else None,
      "distance_source": "odometer" if use_odometer else "integrated_v_ego",
      "steer_intervention_count": steering_interventions,
      "steer_interventions_per_100km": round(steer_per_100km, 2) if steer_per_100km is not None else None,
      "steer_intervention_source": "steering_pressed" if uses_pressed_interventions else "torque_delta",
      "total_state_changes": self.engagement_state_changes,
      "raw_disengagement_count": self.disengagement_count,
      "disengagement_count": corrected_disengagement_count,
      "disengagements_per_100km": round(disengagements_per_100km, 2) if disengagements_per_100km is not None else None,
      "disengagements_per_drive_hour": round(disengagements_per_drive_hour, 2) if disengagements_per_drive_hour is not None else None,
      "disengagement_corrections": {
        "version": 1,
        "raw_count": self.disengagement_count,
        "corrected_count": corrected_disengagement_count,
        "manual_shutdown_removed": shutdown_disengagements_removed,
      },
      "speed_bucket_version": 1,
      "speed_buckets": speed_buckets,
      "openpilot_longitudinal": self.openpilot_longitudinal,
      "steering_mode": "trqi_tq_delta" if self.trqi_steering else "legacy_ssc" if self.trqi_steering is False else "unknown",
      "version": self.version,
      "git_branch": self.git_branch,
      "git_commit": self.git_commit,
      "git_commit_date": self.git_commit_date,
      "git_dirty": self.git_dirty,
      "device_type": self.device_type,
      "generated_at": datetime.now(UTC).isoformat(),
    }


def analyze_drive(
  drive: DriveLogSet,
  device_id: str,
  profile: VehicleStatsProfile | None = None,
  log_reader: Any | None = None,
) -> dict[str, Any] | None:
  if log_reader is None:
    from openpilot.tools.lib.logreader import LogReader

    log_reader = LogReader

  analyzer = DriveAnalyzer(drive.name, device_id, profile)
  for path in drive.paths:
    analyzer.process_segment(log_reader(path))
  return analyzer.result()


def _state_default() -> dict[str, Any]:
  return {"version": DRIVE_STATS_SCHEMA_VERSION, "drives": {}}


def _resolve_state_path() -> str:
  global _DRIVE_STATS_STATE_PATH
  if _DRIVE_STATS_STATE_PATH:
    return _DRIVE_STATS_STATE_PATH
  directories = (os.path.join(PERSIST_ROOT, "comma"), os.path.join(PERSIST_ROOT, "teletyped"), "/tmp/comma")
  for directory in directories:
    path = os.path.join(directory, DRIVE_STATS_STATE_FILE)
    if os.path.isfile(path):
      _DRIVE_STATS_STATE_PATH = path
      return path
  for directory in directories:
    try:
      os.makedirs(directory, exist_ok=True)
      path = os.path.join(directory, DRIVE_STATS_STATE_FILE)
      with open(path + ".write-test", "w", encoding="utf-8") as test_file:
        test_file.write("ok")
      os.remove(path + ".write-test")
      _DRIVE_STATS_STATE_PATH = path
      return path
    except OSError:
      continue
  _DRIVE_STATS_STATE_PATH = os.path.join("/tmp", DRIVE_STATS_STATE_FILE)
  return _DRIVE_STATS_STATE_PATH


def _load_state() -> dict[str, Any]:
  try:
    with open(_resolve_state_path(), encoding="utf-8") as state_file:
      state = json.load(state_file)
    if isinstance(state, dict) and state.get("version") == DRIVE_STATS_SCHEMA_VERSION and isinstance(state.get("drives"), dict):
      return state
  except (OSError, ValueError):
    pass
  return _state_default()


def _save_state(state: dict[str, Any]) -> None:
  path = _resolve_state_path()
  temp_path = path + ".tmp"
  with open(temp_path, "w", encoding="utf-8") as state_file:
    json.dump(state, state_file, sort_keys=True)
  os.replace(temp_path, path)


def upload_drive_stats(stats: dict[str, Any], headers: dict[str, str]) -> bool:
  response = http_post(DRIVE_STATS_UPLOAD_PATH, json=stats, headers=headers, timeout=30)
  if response.status_code == 409:
    return True
  response.raise_for_status()
  return True


def drive_stats_step(device_id: str, headers: dict[str, str]) -> None:
  if not DRIVE_STATS_ENABLED:
    return

  state = _load_state()
  drive_states: dict[str, Any] = state["drives"]
  processed = 0
  current_profile = _current_vehicle_profile()

  for drive in discover_drive_logs():
    previous = drive_states.get(drive.name, {})
    if previous.get("fingerprint") == drive.fingerprint and previous.get("status") in {"uploaded", "unsupported", "error"}:
      continue

    stats = previous.get("stats") if previous.get("fingerprint") == drive.fingerprint else None
    try:
      last_attempt_at = float(previous.get("last_attempt_at") or 0.0)
    except (TypeError, ValueError):
      last_attempt_at = 0.0
    elapsed_since_attempt = time.time() - last_attempt_at
    if isinstance(stats, dict) and DRIVE_STATS_RETRY_INTERVAL > 0 and 0 <= elapsed_since_attempt < DRIVE_STATS_RETRY_INTERVAL:
      continue
    try:
      if not isinstance(stats, dict):
        stats = analyze_drive(drive, device_id, current_profile)
        if stats is None:
          drive_states[drive.name] = {"fingerprint": drive.fingerprint, "status": "unsupported"}
          _save_state(state)
          continue
        drive_states[drive.name] = {"fingerprint": drive.fingerprint, "status": "pending_upload", "stats": stats}
        _save_state(state)

      drive_states[drive.name]["last_attempt_at"] = time.time()
      _save_state(state)
      upload_drive_stats(stats, headers)
      drive_states[drive.name] = {
        "fingerprint": drive.fingerprint,
        "status": "uploaded",
        "uploaded_at": datetime.now(UTC).isoformat(),
      }
      _save_state(state)
      log(f"📈 Uploaded engagement stats for {drive.name}.")
    except Exception as e:
      capture_exception(e)
      # Analysis failures are tied to this exact set of files. Upload failures
      # retain the payload and are retried after the configured backoff.
      if not isinstance(stats, dict):
        drive_states[drive.name] = {"fingerprint": drive.fingerprint, "status": "error", "error": str(e)[:300]}
      else:
        drive_states[drive.name]["last_error"] = str(e)[:300]
      _save_state(state)
      log(f"⚠️ Drive stats failed for {drive.name}: {e}", "WARN")
    processed += 1
    if processed >= DRIVE_STATS_MAX_PER_TICK:
      break


__all__ = [
  "DriveAnalyzer",
  "DriveLogSet",
  "HyundaiI30StatsProfile",
  "analyze_drive",
  "discover_drive_logs",
  "drive_stats_step",
  "select_vehicle_profile",
  "upload_drive_stats",
]
