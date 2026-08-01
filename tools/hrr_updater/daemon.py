from __future__ import annotations

import random
import time

from openpilot.tools.hrr_updater.errors import (BootloaderIncompatible, CandidateAuthenticationError,
                                      ExitCode, HrrNotDetected, ImageVerificationError,
                                      ReleaseVerificationError, RollbackOccurred, TrialFailure)
from openpilot.tools.hrr_updater.output import Reporter


def _enabled() -> bool:
  try:
    from openpilot.common.params import Params
    return bool(Params().get_bool("HrrAutoInstall"))
  except Exception:
    return False


def run(args, reporter: Reporter) -> int:
  from openpilot.common.params import Params
  from openpilot.tools.hrr_updater.cli import (_data_root, _flash_local, _install_pending,
                                     _prefetch_release, _set_runtime_status)
  from openpilot.tools.hrr_updater.state import StateStore

  params = Params()
  store = StateStore(_data_root(args.data_dir) / "state.json")
  backoff = 60.0
  next_check = 0.0
  while True:
    try:
      result: int
      if params.get_bool("HrrUpdateDownloadRequest"):
        # The Software panel can request a signed-release check immediately,
        # without waiting for the automatic polling interval or requiring
        # automatic installation to be enabled.
        params.put_bool("HrrUpdateDownloadRequest", False)
        result = ExitCode.OK if _prefetch_release(args, reporter) is not None else ExitCode.NO_UPDATE
      elif params.get_bool("HrrUpdateInProgress"):
        _set_runtime_status("Recovering an interrupted HRR firmware update. Keep ignition on.")
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
          _set_runtime_status("HRR firmware recovered and confirmed. Starting openpilot.", pending=False)
      elif params.get_bool("HrrUpdateStartupHold"):
        result = _install_pending(args, reporter)
        params.put_bool("HrrUpdateStartupHold", False)
        params.put_bool("HrrUpdaterRequested", False)
      elif not _enabled():
        time.sleep(2.0)
        continue
      else:
        args.allow_prerelease = args.allow_prerelease or params.get_bool("HrrAllowPrerelease")
        pending = store.pending()
        if pending is not None:
          params.put_bool("HrrUpdatePending", True)
          if not params.get("HrrUpdateStatus"):
            _set_runtime_status(
              f"Signed HRR {pending['tag']} is ready for the next ignition cycle.", pending=True)
          result = ExitCode.OK
        elif time.monotonic() >= next_check and params.get_bool("IsOffroad") and not params.get_bool("IsOnroad"):
          result = ExitCode.OK if _prefetch_release(args, reporter) is not None else ExitCode.NO_UPDATE
          next_check = time.monotonic() + args.check_interval * random.uniform(0.95, 1.05)
        else:
          time.sleep(2.0)
          continue
      if result in (ExitCode.OK, ExitCode.NO_UPDATE, ExitCode.FAILED_RELEASE_SUPPRESSED):
        backoff = 60.0
        time.sleep(2.0)
      else:
        time.sleep(backoff)
        backoff = min(backoff * 2.0, args.check_interval)
    except HrrNotDetected as exc:
      # HRR may be absent or slow on this ignition cycle. No bootloader command
      # was sent, so release the bounded hold and retry the staged image on a
      # later ignition rather than preventing this drive.
      reporter.event("auto", f"{exc}; allowing ONROAD without changing HRR")
      _set_runtime_status(f"{exc}; firmware was not changed.", pending=True)
      params.put_bool("HrrUpdateStartupHold", False)
      params.put_bool("HrrUpdaterRequested", False)
      time.sleep(2.0)
    except (BootloaderIncompatible, CandidateAuthenticationError, ReleaseVerificationError, ImageVerificationError,
            RollbackOccurred, TrialFailure) as exc:
      # Permanent candidate-specific failures must not block or repeat on every
      # ignition. _flash_local records trial/rollback identities separately.
      reporter.event("auto", f"candidate rejected: {exc}")
      store.clear_pending()
      _set_runtime_status(f"HRR update was rejected: {exc}", pending=False)
      if not params.get_bool("HrrUpdateInProgress"):
        params.put_bool("HrrUpdateStartupHold", False)
        params.put_bool("HrrUpdaterRequested", False)
      time.sleep(2.0)
    except KeyboardInterrupt:
      return int(ExitCode.OK)
    except Exception as exc:
      reporter.event("auto", f"attempt failed: {exc}; retrying with backoff")
      _set_runtime_status(f"HRR update attempt failed: {exc}")
      if params.get_bool("HrrUpdateStartupHold") and not params.get_bool("HrrUpdateInProgress"):
        params.put_bool("HrrUpdateStartupHold", False)
        params.put_bool("HrrUpdaterRequested", False)
      time.sleep(backoff)
      backoff = min(backoff * 2.0, args.check_interval)


def main() -> int:
  from openpilot.tools.hrr_updater.cli import build_parser
  args = build_parser().parse_args(["auto"])
  return run(args, Reporter(plain=True))


if __name__ == "__main__":
  raise SystemExit(main())
