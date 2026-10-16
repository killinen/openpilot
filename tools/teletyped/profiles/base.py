"""Stateless vehicle configuration and decoding hooks for drive statistics."""

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Literal

SteeringSource = Literal["torque_delta", "steering_pressed"]


@dataclass(frozen=True)
class SteeringStatsConfig:
  # Collect all supported sources; logged metadata can select one after samples
  # have already arrived (for example, legacy actuator feedback late in a log).
  sources: tuple[SteeringSource, ...] = ()
  filter_window: int = 10
  intervention_threshold: float = 1.5
  resolution_threshold: float = 0.3
  intervention_timeout_ns: int = 10 * 1_000_000_000
  engagement_buffer_ns: int = 3 * 1_000_000_000


class VehicleStatsProfile:
  """Configure a car without changing discovery, analysis or upload code.

  Profiles are shared between routes. Keep route state in DriveAnalyzer and
  use the supplied logged parameters rather than reading current device Params.
  """

  name = "unknown"
  car_names: frozenset[str] = frozenset()
  fingerprints: frozenset[str] = frozenset()
  steering_mode = "unknown"
  steering = SteeringStatsConfig()

  def matches(self, car_name: str, fingerprint: str) -> bool:
    return bool(self.car_names or self.fingerprints) and (
      not self.car_names or car_name.strip().lower() in self.car_names
    ) and (not self.fingerprints or fingerprint.strip().upper() in self.fingerprints)

  @property
  def supports_steering_interventions(self) -> bool:
    return bool(self.steering.sources)

  def decode_odometer_km(self, source: int, address: int, data: bytes) -> float | None:
    # With no odometer decoder, the analyzer integrates logged carState.vEgo.
    return None

  def has_legacy_steering_feedback(self, source: int, address: int) -> bool:
    return False

  def steering_values(self, car_state: Any) -> tuple[float, float]:
    return float(getattr(car_state, "steeringTorque", 0.0)), float(getattr(car_state, "steeringTorqueEps", 0.0))

  def intervention_source(self, params: Mapping[str, bytes], saw_legacy_feedback: bool) -> SteeringSource | None:
    return self.steering.sources[0] if self.steering.sources else None

  def get_steering_mode(self, params: Mapping[str, bytes]) -> str:
    return self.steering_mode
