from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from openpilot.tools.hrr_updater import UPDATER_VERSION
from openpilot.tools.hrr_updater.constants import DEFAULT_PANDA_BUS, DEFAULT_PUBLIC_KEY, SLOT_A, SLOT_B, SLOT_NAMES
from openpilot.tools.hrr_updater.errors import (BootloaderIncompatible, ExitCode, HrrNotDetected, HrrUpdaterError,
                                      ReleaseVerificationError, RollbackOccurred, TrialFailure)
from openpilot.tools.hrr_updater.manifest import Version, load_public_key, load_verified_artifacts
from openpilot.tools.hrr_updater.openpilot_transport import OpenpilotPandaTransport
from openpilot.tools.hrr_updater.output import Reporter
from openpilot.tools.hrr_updater.release import ArtifactCache, GitHubReleaseClient, ReleaseCandidate
from openpilot.tools.hrr_updater.safety import OpenpilotSafetyGate
from openpilot.tools.hrr_updater.state import StateStore
from openpilot.tools.hrr_updater.updater import HrrUpdater


def _data_root(value: str | None) -> Path:
  return Path(value if value is not None else os.getenv("HRR_UPDATER_DATA_DIR", "/data/hrr_updater"))


def _state_store(args: argparse.Namespace) -> StateStore:
  return StateStore(_data_root(args.data_dir) / "state.json")


def _set_runtime_status(message: str, *, pending: bool | None = None) -> None:
  try:
    from openpilot.common.params import Params
    params = Params()
    params.put("HrrUpdateStatus", message)
    if pending is not None:
      params.put_bool("HrrUpdatePending", pending)
  except Exception:
    pass


def _new_runtime(args: argparse.Namespace, reporter: Reporter):
  safety = OpenpilotSafetyGate(args.bus)
  transport = OpenpilotPandaTransport()
  return transport, safety, HrrUpdater(transport, safety, args.bus, reporter,
                                      chunk_size=args.chunk_size, frame_pacing=args.frame_pacing)


def _version_key(version: Version) -> tuple[int, int, int, int, str]:
  return version.major, version.minor, version.patch, 1 if not version.prerelease else 0, version.prerelease


def _select_candidate(candidates: list[ReleaseCandidate], current: Version | None,
                      explicit_tag: str | None) -> ReleaseCandidate | None:
  for candidate in candidates:
    if explicit_tag or current is None or _version_key(candidate.version) > _version_key(current):
      return candidate
  return None


def _prefetch_release(args: argparse.Namespace, reporter: Reporter) -> ReleaseCandidate | None:
  """Download and verify both slots without requiring ignition-powered HRR."""
  public_key = load_public_key(args.public_key)
  client = GitHubReleaseClient(args.repository)
  candidates = client.candidates(public_key, tag=args.release, allow_prerelease=args.allow_prerelease)
  updater_version = Version.parse(UPDATER_VERSION)
  candidates = [candidate for candidate in candidates
                if _version_key(Version.parse(candidate.metadata["minimum_updater_version"])) <=
                _version_key(updater_version)]
  store = _state_store(args)
  installed_text = store.installed_version()
  installed = Version.parse(installed_text) if installed_text else None
  candidate = _select_candidate(candidates, installed, args.release)
  if candidate is None:
    reporter.event("prefetch", "no newer signed release to stage")
    _set_runtime_status("HRR firmware is already up to date.", pending=False)
    return None

  cache = ArtifactCache(_data_root(args.data_dir) / "cache")
  slots: dict[str, dict[str, str]] = {}
  reporter.event("prefetch", f"downloading complete A/B candidate {candidate.tag}")
  for slot in (SLOT_A, SLOT_B):
    manifest_path, image_path, manifest = client.download_verified_slot(candidate, slot, cache, public_key)
    slots[SLOT_NAMES[slot]] = {
      "manifest": str(manifest_path), "image": str(image_path),
      "image_identity": manifest.identity,
    }
  store.set_pending(candidate.tag, str(candidate.version), slots)
  _set_runtime_status(
    f"Signed HRR {candidate.tag} is downloaded and ready. It will be checked when ignition powers HRR.",
    pending=True)
  reporter.event("prefetch", f"verified both slots for {candidate.tag}; no HRR CAN command was sent")
  return candidate


def _check_release_without_hrr(args: argparse.Namespace, reporter: Reporter) -> ReleaseCandidate | None:
  public_key = load_public_key(args.public_key)
  candidates = GitHubReleaseClient(args.repository).candidates(
    public_key, tag=args.release, allow_prerelease=args.allow_prerelease)
  updater_version = Version.parse(UPDATER_VERSION)
  candidates = [candidate for candidate in candidates
                if _version_key(Version.parse(candidate.metadata["minimum_updater_version"])) <=
                _version_key(updater_version)]
  installed_text = _state_store(args).installed_version()
  installed = Version.parse(installed_text) if installed_text else None
  candidate = _select_candidate(candidates, installed, args.release)
  if candidate is None:
    reporter.event("check", f"no newer signed release; last observed HRR is v{installed_text or 'unknown'}")
  else:
    reporter.event("check", f"{candidate.tag} is available for offline A/B staging")
  return candidate


def _install_pending(args: argparse.Namespace, reporter: Reporter) -> int:
  store = _state_store(args)
  pending = store.pending()
  if pending is None:
    _set_runtime_status("No staged HRR candidate is available.", pending=False)
    return ExitCode.NO_UPDATE
  version = Version.parse(str(pending["version"]))
  _set_runtime_status(f"HRR v{version} preflight: reading installed firmware before ONROAD.")

  transport, _, updater = _new_runtime(args, reporter)
  try:
    app = updater.read_application(timeout=4.0)
  finally:
    transport.close()
  if app is None:
    raise HrrNotDetected(f"ignition is active but HRR was not detected on Panda bus {args.bus}")
  store.set_installed_version(str(app.version))
  if _version_key(version) <= _version_key(app.version):
    store.clear_pending()
    _set_runtime_status(f"HRR v{app.version} is already current; starting openpilot.", pending=False)
    return ExitCode.NO_UPDATE

  target = _target_slot(app, None)
  try:
    slot = pending["slots"][SLOT_NAMES[target]]
    args.manifest, args.image = Path(slot["manifest"]), Path(slot["image"])
  except (KeyError, TypeError) as exc:
    raise ReleaseVerificationError("staged candidate does not contain the required inactive slot") from exc
  manifest, _ = load_verified_artifacts(args.manifest, args.image, args.public_key)
  if manifest.required_bootloader_version > app.bootloader_version:
    required = manifest.required_bootloader_version
    installed = app.bootloader_version
    raise BootloaderIncompatible(
      f"HRR v{version} requires bootloader {(required >> 16) & 0xff}.{(required >> 8) & 0xff}.{required & 0xff}; "
      + f"installed {(installed >> 16) & 0xff}.{(installed >> 8) & 0xff}.{installed & 0xff}. "
      +
      "Use DFU/service; no bootloader CAN update was attempted")
  _set_runtime_status(
    f"Installing HRR v{version} to inactive slot {SLOT_NAMES[target]}. Keep ignition on.")
  result = _flash_local(args, reporter)
  if result == ExitCode.OK:
    store.set_installed_version(str(version))
    store.clear_pending()
    _set_runtime_status(f"HRR v{version} confirmed. Starting openpilot.", pending=False)
  elif result == ExitCode.FAILED_RELEASE_SUPPRESSED:
    store.clear_pending()
    _set_runtime_status(f"HRR v{version} previously failed and will not be retried automatically.", pending=False)
  return result


def _target_slot(app, boot) -> int:
  active = app.active_slot if app is not None else boot.confirmed_slot
  if active == SLOT_A:
    return SLOT_B
  if active == SLOT_B:
    return SLOT_A
  # This is the existing HRR bootloader's recovery policy when no confirmed
  # application exists. The bootloader remains authoritative and rechecks it.
  return SLOT_A


def _discover_with_optional_lease(updater: HrrUpdater, safety: OpenpilotSafetyGate):
  app = updater.read_application()
  if app is not None:
    return app, None
  preserve = safety.in_progress
  safety.acquire()
  try:
    boot = updater.read_bootloader()
    if boot is None:
      raise HrrNotDetected(f"HRR not detected on Panda bus {updater.bus}")
    if not preserve:
      safety.mark_in_progress()
      preserve = True
    return None, boot
  finally:
    safety.release(preserve_in_progress=preserve)


def _info(args: argparse.Namespace, reporter: Reporter) -> int:
  transport, safety, updater = _new_runtime(args, reporter)
  try:
    app, boot = _discover_with_optional_lease(updater, safety)
    if app is not None:
      reporter.event("info", f"firmware={app.version} build=0x{app.build_identity:08x} " +
                     f"slot={SLOT_NAMES.get(app.active_slot, '?')} confirmed={app.confirmed} " +
                     f"bootloader=0x{app.bootloader_version:08x} security={app.security_version}")
    else:
      reporter.event("info", f"bootloader=0x{boot.bootloader_version:08x} " +
                     f"confirmed_slot={SLOT_NAMES.get(boot.confirmed_slot, 'none')} " +
                     f"pending_slot={SLOT_NAMES.get(boot.pending_slot, 'none')} max_chunk={boot.max_chunk}")
    return ExitCode.OK
  finally:
    transport.close()


def _flash_local(args: argparse.Namespace, reporter: Reporter, *, recover: bool = False) -> int:
  state = None
  manifest_path = getattr(args, "manifest", None)
  image_path = getattr(args, "image", None)
  if recover and (manifest_path is None or image_path is None):
    state = StateStore(_data_root(args.data_dir) / "state.json")
    active = state.active()
    if active is None:
      raise HrrNotDetected("no persisted candidate is available to recover")
    manifest_path, image_path = Path(active["manifest"]), Path(active["image"])
  if manifest_path is None or image_path is None:
    raise ValueError("--manifest and --image are required")
  manifest, image = load_verified_artifacts(manifest_path, image_path, args.public_key)
  if args.dry_run:
    reporter.event("dry-run", f"verified local slot {SLOT_NAMES[manifest.slot]} candidate; no CAN sent")
    return ExitCode.OK
  state = state or StateStore(_data_root(args.data_dir) / "state.json")
  if state.is_failed(str(manifest.version), manifest.identity) and not args.force_retry_failed_release:
    reporter.event("suppressed", "this exact candidate previously failed trial/rollback")
    return ExitCode.FAILED_RELEASE_SUPPRESSED
  state.set_active(manifest_path, image_path, str(manifest.version), manifest.identity)
  transport, safety, updater = _new_runtime(args, reporter)
  try:
    try:
      result = updater.flash(manifest, image, no_reset=args.no_reset, recover=recover)
    except (RollbackOccurred, TrialFailure) as exc:
      state.record_failed(str(manifest.version), manifest.identity, str(exc))
      raise
    if result is not None:
      state.clear_active()
    return ExitCode.OK
  finally:
    transport.close()


def _status(args: argparse.Namespace, reporter: Reporter) -> int:
  state = StateStore(_data_root(args.data_dir) / "state.json").load()
  active = state.get("active_candidate")
  failed = state.get("failed_candidate")
  pending = state.get("pending_candidate")
  reporter.event("status", f"pending_candidate={pending or 'none'} active_candidate={active or 'none'} " +
                 f"failed_candidate={failed or 'none'}")
  return _info(args, reporter)


def _add_common(parser: argparse.ArgumentParser) -> None:
  parser.add_argument("--bus", type=int, default=DEFAULT_PANDA_BUS, help="Panda/openpilot bus number")
  parser.add_argument("--public-key", type=Path, default=DEFAULT_PUBLIC_KEY)
  parser.add_argument("--data-dir")
  parser.add_argument("--chunk-size", type=int, default=128)
  parser.add_argument("--frame-pacing", type=float, default=0.003,
                      help="minimum seconds between ISO-TP consecutive frames")
  parser.add_argument("--json", action="store_true")
  parser.add_argument("--verbose", action="store_true")


def _add_release(parser: argparse.ArgumentParser) -> None:
  parser.add_argument("--repository", default=os.getenv("HRR_GITHUB_REPOSITORY", "killinen/HRR"))
  parser.add_argument("--release")
  parser.add_argument("--allow-prerelease", action="store_true")


def _add_install(parser: argparse.ArgumentParser) -> None:
  parser.add_argument("--dry-run", action="store_true")
  parser.add_argument("--no-reset", action="store_true")
  parser.add_argument("--force-retry-failed-release", action="store_true")


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(prog="python3 -m tools.hrr_updater.cli")
  commands = parser.add_subparsers(dest="command", required=True)
  for name in ("info", "status"):
    _add_common(commands.add_parser(name))
  for name in ("check", "download"):
    command = commands.add_parser(name)
    _add_common(command)
    _add_release(command)
  flash = commands.add_parser("flash")
  _add_common(flash)
  _add_install(flash)
  flash.add_argument("--manifest", type=Path, required=True)
  flash.add_argument("--image", type=Path, required=True)
  update = commands.add_parser("update")
  _add_common(update)
  _add_release(update)
  _add_install(update)
  recover = commands.add_parser("recover")
  _add_common(recover)
  _add_install(recover)
  recover.add_argument("--manifest", type=Path)
  recover.add_argument("--image", type=Path)
  auto = commands.add_parser("auto")
  _add_common(auto)
  _add_release(auto)
  _add_install(auto)
  auto.add_argument("--check-interval", type=float, default=6 * 60 * 60)
  return parser


def main(argv: list[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  reporter = Reporter(json_output=args.json, plain=not sys.stderr.isatty())
  try:
    if args.command == "info":
      return int(_info(args, reporter))
    if args.command == "status":
      return int(_status(args, reporter))
    if args.command == "check":
      return int(ExitCode.OK if _check_release_without_hrr(args, reporter) is not None else ExitCode.NO_UPDATE)
    if args.command == "download":
      return int(ExitCode.OK if _prefetch_release(args, reporter) is not None else ExitCode.NO_UPDATE)
    if args.command == "flash":
      return int(_flash_local(args, reporter))
    if args.command == "update":
      # Ignition-powered HRR requires a split transaction: stage both slots
      # while parked, then let hardwared's next ignition startup hold select
      # and install the inactive slot without network access.
      try:
        from openpilot.common.params import Params
        if Params().get_bool("HrrUpdateStartupHold"):
          return int(_install_pending(args, reporter))
      except Exception:
        pass
      return int(ExitCode.OK if _prefetch_release(args, reporter) is not None else ExitCode.NO_UPDATE)
    if args.command == "recover":
      return int(_flash_local(args, reporter, recover=True))
    if args.command == "auto":
      from openpilot.tools.hrr_updater.daemon import run
      return run(args, reporter)
  except HrrUpdaterError as exc:
    reporter.event("failed", str(exc), exit_code=int(exc.exit_code))
    return int(exc.exit_code)
  except (OSError, ValueError) as exc:
    reporter.event("failed", str(exc), exit_code=int(ExitCode.PROGRAM_FAILURE))
    return int(ExitCode.PROGRAM_FAILURE)
  return int(ExitCode.PROGRAM_FAILURE)


if __name__ == "__main__":
  raise SystemExit(main())
