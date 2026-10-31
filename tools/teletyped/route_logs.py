"""Shared route discovery for offroad log consumers."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import os

from openpilot.tools.teletyped.helper import REALDATA_DIR
from openpilot.tools.teletyped.label_utils import drive_base_name, is_drive_label


@dataclass(frozen=True)
class DriveLogSet:
  name: str
  paths: tuple[str, ...]
  fingerprint: str


def _segment_number(path: str) -> int:
  name = os.path.basename(os.path.dirname(path))
  suffix = name.rsplit("--", 1)[-1]
  return int(suffix) if suffix.isdigit() else -1


def _preferred_log_path(segment_path: str, rlogs_only: bool = False) -> str | None:
  basenames = ("rlog", "rlog.bz2") if rlogs_only else ("rlog", "rlog.bz2", "qlog", "qlog.bz2")
  for basename in basenames:
    candidate = os.path.join(segment_path, basename)
    if os.path.isfile(candidate):
      return candidate
  return None


def discover_drive_logs(realdata_dir: str = REALDATA_DIR, *, rlogs_only: bool = False) -> list[DriveLogSet]:
  grouped: dict[str, list[str]] = {}
  if not os.path.isdir(realdata_dir):
    return []

  for entry in sorted(os.listdir(realdata_dir)):
    entry_path = os.path.join(realdata_dir, entry)
    if entry == "boot" or not os.path.isdir(entry_path) or not is_drive_label(entry):
      continue
    log_path = _preferred_log_path(entry_path, rlogs_only)
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
