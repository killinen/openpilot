from __future__ import annotations

import random
import time

from openpilot.tools.trqi_updater.errors import (BootloaderIncompatible, CandidateAuthenticationError,
                                      ExitCode, TrqiNotDetected, ImageVerificationError,
                                      ReleaseVerificationError, RollbackOccurred, TrialFailure)
from openpilot.tools.trqi_updater.output import Reporter
from openpilot.tools.trqi_updater.ownership import UpdateOwnership


def _enabled() -> bool:
  try:
    from openpilot.common.params import Params
    return bool(Params().get_bool("TrqiAutoInstall"))
  except Exception:
    return False


def run(args, reporter: Reporter) -> int:
  from openpilot.common.params import Params
  from openpilot.tools.trqi_updater.cli import (_data_root, _flash_local, _install_pending,
                                     _prefetch_release, _set_runtime_status)
  from openpilot.tools.trqi_updater.state import StateStore

  params = Params()
  store = StateStore(_data_root(args.data_dir) / "state.json")
  owner = UpdateOwnership()
  initialized = False
  def pause(seconds):
    # Idle/backoff time must not prevent a manual command from acquiring.
    owner.release()
    time.sleep(seconds)
  backoff = 60.0
  next_check = 0.0
  while True:
    # Failure to open the lock must also fail closed, before any status write.
    if not owner.acquire():
      time.sleep(2.0)
      continue
    try:
      if not initialized:
        store.clear_stale_active()
        initialized = True
      result: int
      if params.get_bool("TrqiUpdateDownloadRequest"):
        # The Software panel can request a signed-release check immediately,
        # without waiting for the automatic polling interval or requiring
        # automatic installation to be enabled.
        params.put_bool("TrqiUpdateDownloadRequest", False)
        result = ExitCode.OK if _prefetch_release(args, reporter) is not None else ExitCode.NO_UPDATE
      elif params.get_bool("TrqiUpdateInProgress"):
        _set_runtime_status("Recovering an interrupted TRQI firmware update. Keep ignition on.")
        reporter.event("auto", "persisted update detected; attempting durable recovery")
        force_retry = args.force_retry_failed_release
        args.force_retry_failed_release = True
        try:
          result = _flash_local(args, reporter, recover=True)
        finally:
          args.force_retry_failed_release = force_retry
        if result == ExitCode.OK:
          pending = store.pending()
          if pending is not None:
            store.set_installed_version(str(pending["version"]))
            store.clear_pending()
          _set_runtime_status("TRQI firmware recovered and confirmed. Starting openpilot.", pending=False)
      elif params.get_bool("TrqiUpdateStartupHold"):
        result = _install_pending(args, reporter)
        params.put_bool("TrqiUpdateStartupHold", False)
        params.put_bool("TrqiUpdaterRequested", False)
      elif not _enabled():
        pause(2.0)
        continue
      else:
        args.allow_prerelease = args.allow_prerelease or params.get_bool("TrqiAllowPrerelease")
        pending = store.pending()
        if pending is not None:
          params.put_bool("TrqiUpdatePending", True)
          if not params.get("TrqiUpdateStatus"):
            _set_runtime_status(
              f"Signed TRQI {pending['tag']} is ready for the next ignition cycle.", pending=True)
          result = ExitCode.OK
        elif time.monotonic() >= next_check and params.get_bool("IsOffroad") and not params.get_bool("IsOnroad"):
          result = ExitCode.OK if _prefetch_release(args, reporter) is not None else ExitCode.NO_UPDATE
          next_check = time.monotonic() + args.check_interval * random.uniform(0.95, 1.05)
        else:
          pause(2.0)
          continue
      if result in (ExitCode.OK, ExitCode.NO_UPDATE, ExitCode.FAILED_RELEASE_SUPPRESSED):
        backoff = 60.0
        pause(2.0)
      else:
        pause(backoff)
        backoff = min(backoff * 2.0, args.check_interval)
    except TrqiNotDetected as exc:
      # TRQI may be absent or slow on this ignition cycle. No bootloader command
      # was sent, so release the bounded hold and retry the staged image on a
      # later ignition rather than preventing this drive.
      reporter.event("auto", f"{exc}; allowing ONROAD without changing TRQI")
      _set_runtime_status(f"{exc}; firmware was not changed.", pending=True)
      params.put_bool("TrqiUpdateStartupHold", False)
      params.put_bool("TrqiUpdaterRequested", False)
      pause(2.0)
    except (BootloaderIncompatible, CandidateAuthenticationError, ReleaseVerificationError, ImageVerificationError,
            RollbackOccurred, TrialFailure) as exc:
      # Permanent candidate-specific failures must not block or repeat on every
      # ignition. _flash_local records trial/rollback identities separately.
      reporter.event("auto", f"candidate rejected: {exc}")
      store.clear_active()
      store.clear_pending()
      _set_runtime_status(f"TRQI update was rejected: {exc}", pending=False)
      if not params.get_bool("TrqiUpdateInProgress"):
        params.put_bool("TrqiUpdateStartupHold", False)
        params.put_bool("TrqiUpdaterRequested", False)
      pause(2.0)
    except KeyboardInterrupt:
      return int(ExitCode.OK)
    except Exception as exc:
      reporter.event("auto", f"attempt failed: {exc}; retrying with backoff")
      _set_runtime_status(f"TRQI update attempt failed: {exc}")
      if params.get_bool("TrqiUpdateStartupHold") and not params.get_bool("TrqiUpdateInProgress"):
        params.put_bool("TrqiUpdateStartupHold", False)
        params.put_bool("TrqiUpdaterRequested", False)
      pause(backoff)
      backoff = min(backoff * 2.0, args.check_interval)
    finally:
      owner.release()


def main() -> int:
  from openpilot.tools.trqi_updater.cli import build_parser
  args = build_parser().parse_args(["auto"])
  return run(args, Reporter(plain=True, publish_params=True))


if __name__ == "__main__":
  raise SystemExit(main())
