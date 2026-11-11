from __future__ import annotations

import json
import os
import shutil
import sys
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

LOG_MAX_BYTES = 1_000_000


def _boot_id() -> str:
  try:
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()[:8]
  except OSError:
    return "unknown"


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
  # Persistent diagnostic log. The daemon's stdout is not captured by
  # manager/swaglog, so without this an interrupted update leaves no trace.
  log_path: Path | str | None = None
  log_max_bytes: int = LOG_MAX_BYTES
  last_logged_bucket: int = -1
  last_logged_retries: tuple[int, int] = (0, 0)
  # Also mirror events and failures (not per-chunk progress) to cloudlog so
  # they sit beside pandad/hardwared in swaglog. The file above stays the
  # durable record; swaglog files are small, shared and rotated.
  cloudlog_events: bool = False

  def _cloudlog(self, level: str, **fields: Any) -> None:
    if not self.cloudlog_events:
      return
    try:
      from openpilot.common.swaglog import cloudlog
      safe = {key: (value if isinstance(value, (bool, int, float, str)) or value is None else str(value))
              for key, value in fields.items()}
      cloudlog.event("trqi_updater", error=(level == "error"), **safe)
    except Exception:
      pass

  def log(self, line: str) -> None:
    """Append one line; never raises. Wall clock may be unsynced early after
    boot, so each line also carries boot id and monotonic seconds."""
    if self.log_path is None:
      return
    try:
      path = Path(self.log_path)
      path.parent.mkdir(parents=True, exist_ok=True)
      if path.exists() and path.stat().st_size > self.log_max_bytes:
        os.replace(path, path.with_name(f"{path.stem}.previous{path.suffix}"))
      stamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
      with open(path, "a", encoding="utf-8") as handle:
        handle.write(f"{stamp} boot={_boot_id()} mono={time.monotonic():.1f} pid={os.getpid()} {line}\n")
        handle.flush()
        os.fsync(handle.fileno())
    except Exception:
      pass

  def log_exception(self, stage: str, exc: BaseException) -> None:
    self.log(f"{stage:<18} exception {type(exc).__name__}: {exc}")
    self._cloudlog("error", stage=stage, exception=type(exc).__name__, message=str(exc),
                   traceback="".join(traceback.format_exception(type(exc), exc, exc.__traceback__))[-4000:])
    for text in traceback.format_exception(type(exc), exc, exc.__traceback__):
      for item in text.rstrip().splitlines():
        self.log(f"{stage:<18}   {item}")

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
    details = " ".join(f"{key}={value}" for key, value in sorted(fields.items()))
    self.log(f"{stage:<18} {message}" + (f" [{details}]" if details else ""))
    self._cloudlog("info", stage=stage, message=message, trust=self.trust_profile, **fields)
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
    bucket = int(percent) // 5
    if bucket != self.last_logged_bucket or (retries, timeouts) != self.last_logged_retries or durable == total:
      self.last_logged_bucket = bucket
      self.last_logged_retries = (retries, timeouts)
      self.log(f"{'transfer':<18} durable={durable}/{total} {percent:.1f}% retries={retries} " +
               f"timeouts={timeouts} chunk={chunk_size} ack_age_ms={round(ack_age * 1000)} " +
               f"kib_s={throughput:.2f}")
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
