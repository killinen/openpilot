"""One device-wide owner for CAN updates and their shared UI status.

The lock is deliberately independent of --data-dir and CAN bus: all updater
instances share Panda's lease and the same UI Params. Never unlink this file;
flock releases ownership automatically when a process exits or crashes.
"""
import fcntl
from pathlib import Path

LOCK_PATH = Path("/data/trqi_updater/owner.lock")


class UpdateOwnership:
  def __init__(self, path=None):
    self.path = Path(path) if path is not None else LOCK_PATH
    self.file = None

  def acquire(self):
    if self.file is not None:
      return True
    self.path.parent.mkdir(parents=True, exist_ok=True)
    handle = self.path.open("a+")
    try:
      fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      handle.close()
      return False
    except BaseException:
      handle.close()
      raise
    self.file = handle
    return True

  def release(self):
    if self.file is not None:
      self.file.close()
      self.file = None
