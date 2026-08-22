from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class FailedCandidate:
  version: str
  image_identity: str
  reason: str
  timestamp: float


class StateStore:
  def __init__(self, path: Path):
    self.path = path

  def load(self) -> dict[str, Any]:
    try:
      value: Any = json.loads(self.path.read_text())
      return value if isinstance(value, dict) else {}
    except FileNotFoundError:
      return {}
    except (OSError, json.JSONDecodeError):
      return {}

  def save(self, value: dict[str, Any]) -> None:
    self.path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{self.path.name}.", dir=self.path.parent)
    try:
      with os.fdopen(fd, "w", encoding="utf-8") as output:
        json.dump(value, output, sort_keys=True, separators=(",", ":"))
        output.flush()
        os.fsync(output.fileno())
      os.replace(temporary_name, self.path)
    finally:
      try:
        os.unlink(temporary_name)
      except FileNotFoundError:
        pass

  def record_failed(self, version: str, image_identity: str, reason: str) -> None:
    value = self.load()
    value["failed_candidate"] = asdict(FailedCandidate(version, image_identity, reason, time.time()))
    # A recorded trial/rollback failure is definitive, not an interrupted
    # transfer that can still be recovered from active_candidate.
    value.pop("active_candidate", None)
    self.save(value)

  def is_failed(self, version: str, image_identity: str) -> bool:
    failed = self.load().get("failed_candidate", {})
    return bool(failed.get("version") == version and failed.get("image_identity") == image_identity)

  def set_active(self, manifest: Path, image: Path, version: str, image_identity: str, *,
                 trust_profile: str = "production") -> None:
    value = self.load()
    value["active_candidate"] = {
      "manifest": str(manifest), "image": str(image), "version": version,
      "image_identity": image_identity, "trust_profile": trust_profile,
    }
    self.save(value)

  def active(self) -> dict[str, str] | None:
    candidate = self.load().get("active_candidate")
    return candidate if isinstance(candidate, dict) else None

  def clear_active(self) -> None:
    value = self.load()
    value.pop("active_candidate", None)
    self.save(value)

  def clear_stale_active(self) -> bool:
    """Clear legacy recovery state for a candidate already recorded as failed."""
    value = self.load()
    active = value.get("active_candidate")
    failed = value.get("failed_candidate")
    if not isinstance(active, dict) or not isinstance(failed, dict):
      return False
    if (active.get("version") != failed.get("version") or
        active.get("image_identity") != failed.get("image_identity")):
      return False
    value.pop("active_candidate", None)
    self.save(value)
    return True

  def set_pending(self, tag: str, version: str, slots: dict[str, dict[str, str]], *,
                  trust_profile: str = "production", force_install: bool = False) -> None:
    value = self.load()
    value["pending_candidate"] = {"tag": tag, "version": version, "slots": slots,
                                  "trust_profile": trust_profile, "force_install": force_install}
    self.save(value)

  def pending(self) -> dict[str, Any] | None:
    candidate = self.load().get("pending_candidate")
    return candidate if isinstance(candidate, dict) else None

  def clear_pending(self) -> None:
    value = self.load()
    value.pop("pending_candidate", None)
    self.save(value)

  def set_installed_version(self, version: str) -> None:
    value = self.load()
    value["installed_version"] = version
    self.save(value)

  def installed_version(self) -> str | None:
    version = self.load().get("installed_version")
    return version if isinstance(version, str) else None
