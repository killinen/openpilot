from __future__ import annotations

import json
import os
import shutil
import sys
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Reporter:
  json_output: bool = False
  plain: bool = False
  stream: Any = sys.stderr
  started_at: float = field(default_factory=time.monotonic)
  last_progress_line: str = ""
  publish_params: bool = False
  trust_profile: str = "production"
  last_param_publish: float = 0.0
  context: dict[str, Any] = field(default_factory=dict)

  def _publish(self, payload: dict[str, Any], *, force: bool = False) -> None:
    if not self.publish_params:
      return
    now = time.monotonic()
    if not force and now - self.last_param_publish < 0.25:
      return
    try:
      from openpilot.common.params import Params
      params = Params()
      payload = {"trust": self.trust_profile, **payload}
      params.put("TrqiUpdateProgress", json.dumps(payload, sort_keys=True, separators=(",", ":")))
      if "message" in payload:
        params.put("TrqiUpdateStatus", str(payload["message"]))
      self.last_param_publish = now
    except Exception:
      pass

  @property
  def interactive(self) -> bool:
    return (not self.json_output and not self.plain and self.stream.isatty() and
            os.getenv("NO_COLOR") is None and os.getenv("TERM") != "dumb")

  def event(self, stage: str, message: str, **fields: Any) -> None:
    timestamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    self.context.update(fields)
    self._publish({**self.context, "stage": stage, "message": message}, force=True)
    if self.json_output:
      print(json.dumps({"timestamp": timestamp, "stage": stage, "message": message, **fields},
                       sort_keys=True), file=sys.stdout, flush=True)
      return
    prefix = "●" if self.interactive else ("[TRQI TEST]" if self.trust_profile == "test" else "[TRQI]")
    print(f"{timestamp} {prefix} {stage:<18} {message}", file=self.stream, flush=True)

  def progress(self, durable: int, total: int, *, retries: int, timeouts: int, chunk_size: int,
               elapsed: float, ack_age: float = 0.0) -> None:
    percent = 100.0 if total == 0 else durable * 100.0 / total
    throughput = durable / max(elapsed, 1e-6) / 1024.0
    progress_fields = {**self.context, "stage": "transfer", "message": "Programming inactive slot",
                       "durable_offset": durable, "total": total, "percent": round(percent, 2),
                       "retries": retries, "timeouts": timeouts, "chunk_size": chunk_size,
                       "throughput_kib_s": round(throughput, 2), "ack_age_ms": round(ack_age * 1000)}
    self._publish(progress_fields, force=durable == total)
    if self.json_output:
      self.event("transfer", "durable progress", durable_offset=durable, total=total,
                 percent=round(percent, 2), retries=retries, timeouts=timeouts, chunk_size=chunk_size,
                 throughput_kib_s=round(throughput, 2), ack_age_ms=round(ack_age * 1000))
      return
    width = max(10, min(36, shutil.get_terminal_size((100, 24)).columns - 68))
    filled = min(width, int(width * percent / 100.0))
    bar = "█" * filled + "░" * (width - filled) if self.interactive else "#" * filled + "-" * (width - filled)
    line = (f"Uploading [{bar}] {percent:6.2f}%  durable={durable}/{total}  " +
            f"{throughput:5.1f}KiB/s  chunk={chunk_size}  retry={retries}  timeout={timeouts}")
    if self.interactive:
      print(f"\r\033[2K{line}", end="", file=self.stream, flush=True)
      self.last_progress_line = line
    elif durable == total or not self.last_progress_line or int(percent) // 5 != int(
        (durable - chunk_size) * 100 / max(total, 1)) // 5:
      print(line, file=self.stream, flush=True)
      self.last_progress_line = line

  def finish_progress(self) -> None:
    if self.interactive and self.last_progress_line:
      print(file=self.stream, flush=True)
      self.last_progress_line = ""
