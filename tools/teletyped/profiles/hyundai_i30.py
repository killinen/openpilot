"""Custom i30 odometer and legacy SSC/TRQI steering configuration."""

from collections.abc import Mapping
from typing import Any

from openpilot.tools.teletyped.profiles.base import SteeringSource, SteeringStatsConfig, VehicleStatsProfile


class HyundaiI30StatsProfile(VehicleStatsProfile):
  name = "hyundai_i30_2014"
  fingerprints = frozenset({"HYUNDAI I30 GD 2014", "HYUNDAI_I30_GD_2014"})
  steering = SteeringStatsConfig(sources=("torque_delta", "steering_pressed"))

  def matches(self, car_name: str, fingerprint: str) -> bool:
    return car_name.strip().lower() == "i30" or fingerprint.strip().upper() in self.fingerprints

  def decode_odometer_km(self, source: int, address: int, data: bytes) -> float | None:
    # CLU1.CF_Clu_Odometer: Intel 24-bit value at bit 40, scale 0.1 km.
    if source != 0 or address != 1264 or len(data) < 8:
      return None
    return ((data[7] << 16) | (data[6] << 8) | data[5]) * 0.1

  def has_legacy_steering_feedback(self, source: int, address: int) -> bool:
    return source == 1 and address == 559

  def steering_values(self, car_state: Any) -> tuple[float, float]:
    # Preserve the calibration/sign convention used by opDriveStats.
    return -float(getattr(car_state, "steeringTorque", 0.0)) - 0.2, float(getattr(car_state, "steeringTorqueEps", 0.0))

  @staticmethod
  def _trqi_enabled(params: Mapping[str, bytes]) -> bool | None:
    value = params.get("TRQISteeringToggle")
    return value.strip().lower() in {b"1", b"true", b"yes", b"on"} if value is not None else None

  def active_steering_mode(self, car_params: Any) -> str | None:
    mode = str(getattr(car_params, "steeringMode", ""))
    if not mode:
      return None
    return mode if mode in {"legacy_ssc", "trqi_tq_delta"} else "unknown"

  def intervention_source(self, params: Mapping[str, bytes], saw_legacy_feedback: bool,
                          active_mode: str | None = None) -> SteeringSource | None:
    if active_mode is not None:
      if active_mode == "legacy_ssc":
        return "torque_delta"
      if active_mode == "trqi_tq_delta":
        return "steering_pressed"
      return None
    enabled = self._trqi_enabled(params)
    return "steering_pressed" if enabled is True or (enabled is None and not saw_legacy_feedback) else "torque_delta"

  def get_steering_mode(self, params: Mapping[str, bytes], active_mode: str | None = None) -> str:
    if active_mode is not None:
      return active_mode
    enabled = self._trqi_enabled(params)
    return "trqi_tq_delta" if enabled else "legacy_ssc" if enabled is False else "unknown"
