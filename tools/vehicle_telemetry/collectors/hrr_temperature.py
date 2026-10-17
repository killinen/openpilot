"""LS600h HRR reports and comparison streams for the Conditions/library view."""

from __future__ import annotations

from typing import Any


class HrrTemperatureCollector:
  key = "hrr_temperature"
  schema_version = 2

  def __init__(self) -> None:
    self.samples: list[dict[str, Any]] = []
    self.pending: dict[str, dict[str, Any]] = {}
    self.coverage: dict[int, dict[str, int]] = {}
    self.counter: int | None = None
    self.malformed_frames = 0
    self.repeated_temperature_reports = 0

  def matches(self, car_name: str, fingerprint: str) -> bool:
    return car_name.strip().lower() == "toyota" and fingerprint.strip().upper() in {
      "LEXUS_LS600H",
      "LEXUS LS 600H 2013-17",
    }

  def observe(self, message: Any, elapsed_s: float, segment: int) -> None:
    if message.which() != "can":
      return
    timestamp = int(message.logMonoTime)
    coverage = self.coverage.setdefault(
      segment,
      {
        "segment": segment,
        "first_ns": timestamp,
        "last_ns": timestamp,
        "can_events": 0,
        "hrr_frames": 0,
        "hrr_valid": 0,
        "hrr_invalid": 0,
        "hrr_malformed": 0,
        "hrr_repeated": 0,
      },
    )
    coverage["last_ns"] = timestamp
    coverage["can_events"] += 1
    for frame in message.can:
      bus, address, data = int(frame.src), int(frame.address), bytes(frame.dat)
      metric = {(2, 0x638): "hrr", (0, 0xB4): "speed", (1, 0x3B0): "ambient", (1, 0x39): "engine"}.get((bus, address))
      if metric is None:
        continue
      if metric == "hrr":
        coverage["hrr_frames"] += 1
      minimum_length = {"hrr": 4, "speed": 7, "ambient": 4, "engine": 1}[metric]
      if len(data) < minimum_length or len(data) > 8:
        self.malformed_frames += 1
        if metric == "hrr":
          coverage["hrr_malformed"] += 1
        continue
      row: dict[str, Any] = {"time_s": round(elapsed_s, 6), "time_ns": timestamp, "segment": segment, "metric": metric}
      if metric == "hrr":
        valid, counter = bool(data[2] & 1), data[3]
        if valid and counter == self.counter:
          self.repeated_temperature_reports += 1
          coverage["hrr_repeated"] += 1
          continue
        if valid:
          self.counter = counter
        coverage["hrr_valid" if valid else "hrr_invalid"] += 1
        row.update(value=int.from_bytes(data[:2], "little", signed=True) if valid else None, counter=counter)
        self.samples.append(row)
      else:
        if metric == "speed":
          row["value"] = round(int.from_bytes(data[5:7], "big") * 0.01, 2)
        elif metric == "ambient":
          row.update(value=data[3] - 49, raw=data[3])
        else:
          row["value"] = data[0]
        # Match the desktop library: last companion report in each absolute
        # second, retaining its original timestamp. HRR reports are all retained.
        previous = self.pending.get(metric)
        if previous and (previous["segment"] != segment or previous["time_ns"] // 1_000_000_000 != timestamp // 1_000_000_000):
          self.samples.append(previous)
        self.pending[metric] = row

  def result(self) -> dict[str, Any] | None:
    if not self.coverage:
      return None
    samples = sorted(self.samples + list(self.pending.values()), key=lambda row: (row["time_ns"], row["metric"]))
    return {
      "samples": samples,
      "first_batch": {"coverage": list(self.coverage.values())},
      "signals": {
        "hrr": {"bus": 2, "address": 0x638, "signal": "MCU_Temperature", "unit": "degC", "max_age_s": 1.5},
        "speed": {"bus": 0, "address": 0xB4, "signal": "SPEED", "unit": "km/h", "max_age_s": 2},
        "ambient": {
          "bus": 1,
          "address": 0x3B0,
          "signal": "AMBIENT_TEMP",
          "unit": "degC",
          "max_age_s": 60,
          "provisional": True,
          "decode": "byte3 - 49",
        },
        "engine": {"bus": 1, "address": 0x39, "signal": "ENGINE_TEMP", "unit": "degC", "max_age_s": 5, "tentative": True},
      },
      "malformed_frames": self.malformed_frames,
      "repeated_temperature_reports": self.repeated_temperature_reports,
    }
