"""Small, optional guardian protocol. No openpilot imports or network operations."""
from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time

_lock = threading.Lock()
_state: dict[str, object] = {}


def report(**values):
  path = os.environ.get("TELETYPED_HEALTH_PATH")
  if not path:
    return
  with _lock:
    _state.update(values, run_id=os.environ["TELETYPED_RUN_ID"], release_id=os.environ["TELETYPED_RELEASE_ID"])
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
      json.dump(_state, f)
    os.replace(tmp, path)


def progress(phase="running"):
  report(main_progress=time.monotonic(), phase=phase)


def success(operation):
  report(**{operation + "_ok": time.monotonic()})


def offroad():
  """Unknown manager state suppresses bulk work; opted-in recovery SSH stays available."""
  if not os.environ.get("TELETYPED_DEPLOYMENT_ROOT"):
    return True  # Legacy worker's offroad policy is enforced by manager.
  try:
    path = Path(os.environ.get("TELETYPED_MANAGER_STATE", "/dev/shm/teletyped_manager_state.json"))
    state = json.loads(path.read_text())
    return state.get("offroad") is True and 0 <= time.monotonic() - state["progress"] < 5
  except (OSError, ValueError, KeyError, TypeError):
    return False
