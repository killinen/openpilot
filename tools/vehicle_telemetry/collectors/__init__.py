"""Optional vehicle telemetry modules; transport lives in telemetry.py."""

from __future__ import annotations

import os
from typing import Any, Protocol

from openpilot.tools.vehicle_telemetry.collectors.hrr_temperature import HrrTemperatureCollector
from openpilot.tools.vehicle_telemetry.collectors.trqi_temperature import TrqiTemperatureCollector


class Collector(Protocol):
  key: str
  schema_version: int

  def matches(self, car_name: str, fingerprint: str) -> bool: ...
  def observe(self, message: Any, elapsed_s: float, segment: int) -> None: ...
  def result(self) -> dict[str, Any] | None: ...


COLLECTOR_TYPES = (HrrTemperatureCollector, TrqiTemperatureCollector)


def enabled_collectors() -> list[Collector]:
  return [cls() for cls in COLLECTOR_TYPES if os.environ.get(f"TELETYPED_{cls.key.upper()}", "1").strip().lower() not in {"0", "false", "no", "off"}]
