"""TRQI junction temperature and cabin-condition candidates from recorded logs.

No CANParser, Params, updater, or live vehicle dependencies. Firmware identity is
archived as passive application-info frames; this collector never requests it.
"""

from __future__ import annotations

import math
from typing import Any

SECOND = 1_000_000_000


def crc8(data: bytes) -> int:
  value = 0
  for byte in data:
    value ^= byte
    for _ in range(8):
      value = ((value << 1) ^ 0x07) & 255 if value & 128 else (value << 1) & 255
  return value


class TrqiTemperatureCollector:
  key = "trqi_temperature"
  schema_version = 2

  def __init__(self) -> None:
    self.samples: list[dict[str, Any]] = []
    self.pending: dict[str, dict[str, Any]] = {}
    self.coverage: dict[int, dict[str, int]] = {}
    self.previous: tuple[int, bytes] | None = None
    self.firmware_frames: list[dict[str, Any]] = []
    self.identity: dict[int, bytes] = {}

  def matches(self, car_name: str, fingerprint: str) -> bool:
    return car_name.strip().lower() == "i30" and fingerprint.strip().upper() in {
      "HYUNDAI_I30_GD_2014",
      "HYUNDAI I30 GD 2014",
    }

  def _companion(self, metric: str, value: float, timestamp: int, elapsed: float, segment: int) -> None:
    if not math.isfinite(value):
      return
    row = {"metric": metric, "value": value, "time_ns": timestamp, "time_s": round(elapsed, 6), "segment": segment}
    previous = self.pending.get(metric)
    if previous and (previous["segment"] != segment or previous["time_ns"] // SECOND != timestamp // SECOND):
      self.samples.append(previous)
    self.pending[metric] = row

  def observe(self, message: Any, elapsed_s: float, segment: int) -> None:
    timestamp = int(message.logMonoTime)
    which = message.which()
    coverage = self.coverage.setdefault(
      segment,
      {
        "segment": segment,
        "first_ns": timestamp,
        "last_ns": timestamp,
        "can_events": 0,
        "frames": 0,
        "valid": 0,
        "invalid": 0,
        "malformed": 0,
        "crc_errors": 0,
        "repeated": 0,
        "counter_discontinuities": 0,
      },
    )
    coverage["last_ns"] = timestamp
    if which == "carState":
      state = message.carState
      self._companion("speed", max(0.0, float(state.vEgo)) * 3.6, timestamp, elapsed_s, segment)
      return
    if which == "carControl":
      self._companion("lateral_active", float(bool(message.carControl.latActive)), timestamp, elapsed_s, segment)
      self._companion("steering_demand", abs(float(message.carControl.actuators.steer)), timestamp, elapsed_s, segment)
      return
    if which != "can":
      return
    coverage["can_events"] += 1
    for frame in message.can:
      bus, address, data = int(frame.src), int(frame.address), bytes(frame.dat)
      if bus == 1 and address == 0x245:
        coverage["frames"] += 1
        if len(data) != 8:
          coverage["malformed"] += 1
          continue
        checksum_ok = crc8(bytes((0x45, 0x02)) + data[:7]) == data[7]
        previous = self.previous
        if checksum_ok:
          self.previous = timestamp, data
        if checksum_ok and previous:
          previous_ns, previous_data = previous
          # A 256-second counter wrap or a report after a gap is not a duplicate.
          if 0 <= timestamp - previous_ns <= 2 * SECOND and data == previous_data:
            coverage["repeated"] += 1
            continue
          if timestamp - previous_ns <= 2 * SECOND and data[6] != (previous_data[6] + 1) % 256:
            coverage["counter_discontinuities"] += 1
        valid = checksum_ok and bool(data[4] & 1)
        coverage["valid" if valid else "invalid"] += 1
        coverage["crc_errors"] += not checksum_ok
        self.samples.append(
          {
            "time_s": round(elapsed_s, 6),
            "time_ns": timestamp,
            "segment": segment,
            "metric": "trqi",
            "value": int.from_bytes(data[:2], "little", signed=True) * 0.1 if valid else None,
            "raw": int.from_bytes(data[2:4], "little"),
            "counter": data[6],
            "status": data[4],
            "loop_max_ms": data[5],
            "crc_valid": checksum_ok,
            "sensor_valid": bool(data[4] & 1),
            "frame_hex": data.hex(),
          }
        )
      elif bus == 1 and address == 0x241 and len(data) == 8:
        if crc8(bytes((0x41, 0x02)) + data[:7]) == data[7]:
          bits = int.from_bytes(data, "little")
          self._companion("relay_active", float(bool(bits & (1 << 36))), timestamp, elapsed_s, segment)
      elif bus == 1 and address in (0x639, 0x63A, 0x63B, 0x63C) and len(data) == 8:
        if self.identity.get(address) != data:
          self.identity[address] = data
          self.firmware_frames.append({"time_ns": timestamp, "address": address, "data": data.hex()})
      elif bus == 0 and address == 0x350 and len(data) == 8:
        # Candidate DBC fields: retain provenance and do not call these cabin temperature.
        for metric, value in {
          "outside": data[3] * 0.5 - 40,
          "outside_sensor": data[4] * 0.5 - 40,
          "blower_on": float(bool(data[1] & 8)),
          "ac_request": float(bool(data[1] & 1)),
          "compressor_load": float(data[5] & 7),
        }.items():
          if metric.startswith("outside") and not -40 <= value <= 60:
            continue  # Reserved/sentinel codes remain absent, never plausible temperatures.
          self._companion(metric, value, timestamp, elapsed_s, segment)

  def result(self) -> dict[str, Any] | None:
    if not self.coverage or not any(row["frames"] for row in self.coverage.values()):
      return None  # A legacy SSC i30 is not a TRQI temperature drive.
    return {
      "samples": sorted(self.samples + list(self.pending.values()), key=lambda row: (row["time_ns"], row["metric"])),
      "first_batch": {"coverage": list(self.coverage.values()), "firmware_frames": self.firmware_frames},
      "signals": {
        "trqi": {"bus": 1, "address": 0x245, "decode": "signed_le16 * 0.1", "max_age_s": 1.5},
        "outside": {"bus": 0, "address": 0x350, "decode": "byte3 * 0.5 - 40", "max_age_s": 60, "provisional": True},
        "outside_sensor": {"bus": 0, "address": 0x350, "decode": "byte4 * 0.5 - 40", "max_age_s": 60, "provisional": True},
        "hvac": {"bus": 0, "address": 0x350, "provisional": True},
      },
    }
