from __future__ import annotations

import secrets
import time
from dataclasses import dataclass
from enum import Enum

from openpilot.tools.hrr_updater.constants import (FLAG_UPDATE_SUPPORT, PRODUCT_ID, SLOT_A, SLOT_B,
                                         SLOT_NAMES)
from openpilot.tools.hrr_updater.errors import (BootloaderIncompatible, BootloaderRefused, CandidateAuthenticationError,
                                      HrrNotDetected, RollbackOccurred, TrialFailure, TransportError,
                                      UnsafeStateError)
from openpilot.tools.hrr_updater.manifest import Manifest
from openpilot.tools.hrr_updater.output import Reporter
from openpilot.tools.hrr_updater.protocol import AppInfo, AppInfoCollector, BootInfo, HrrProtocol, UpdateState
from openpilot.tools.hrr_updater.safety import SafetyGate
from openpilot.tools.hrr_updater.transport import CanTransport


class Stage(str, Enum):
  DISCOVER = "discover"
  VERIFY_LOCAL = "verify-local"
  SAFE_STATE = "safe-state"
  ENTER_BOOTLOADER = "enter-bootloader"
  BOOT_INFO = "boot-info"
  START_ERASE = "start-erase"
  TRANSFER = "transfer"
  FINISH = "finish"
  TRIAL = "trial"
  CONFIRM = "confirm"
  SUCCESS = "success"


@dataclass(frozen=True)
class UpdateResult:
  previous: AppInfo | None
  installed: AppInfo
  target_slot: int
  retries: int


class HrrUpdater:
  def __init__(self, transport: CanTransport, safety: SafetyGate, bus: int, reporter: Reporter | None = None,
               *, chunk_size: int = 128, frame_pacing: float = 0.003):
    if bus < 0:
      raise ValueError("Panda bus must be nonnegative")
    self.transport = transport
    self.safety = safety
    self.bus = bus
    self.reporter = reporter or Reporter(plain=True)
    self.requested_chunk_size = chunk_size
    self.protocol = HrrProtocol(transport, bus, frame_pacing=frame_pacing)

  def read_application(self, timeout: float = 1.5) -> AppInfo | None:
    collector = AppInfoCollector(self.bus)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      frame = self.transport.recv(max(0.0, deadline - time.monotonic()))
      if frame is not None:
        collector.add(frame)
        if collector.complete:
          break
    return collector.snapshot()

  def read_bootloader(self, timeout: float = 2.0) -> BootInfo | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      try:
        return self.protocol.get_info()
      except (BootloaderRefused, TransportError):
        time.sleep(0.15)
    return None

  def discover(self, *, query_bootloader: bool = True) -> tuple[AppInfo | None, BootInfo | None]:
    self.reporter.event(Stage.DISCOVER.value, f"listening on Panda bus {self.bus}")
    app = self.read_application()
    if app is not None:
      if app.product_id != PRODUCT_ID:
        raise HrrNotDetected(f"unexpected product ID 0x{app.product_id:08x}")
      self.reporter.event(Stage.DISCOVER.value,
                          f"HRR v{app.version}, slot {SLOT_NAMES.get(app.active_slot, '?')}, " +
                          f"{'CONFIRMED' if app.confirmed else 'TRIAL' if app.trial else 'UNKNOWN'}")
      return app, None
    boot = self.read_bootloader() if query_bootloader else None
    if boot is not None:
      self.reporter.event(Stage.DISCOVER.value, "HRR bootloader/recovery mode detected")
      return None, boot
    if not query_bootloader:
      return None, None
    raise HrrNotDetected(f"HRR not detected on Panda bus {self.bus}")

  @staticmethod
  def _normal_target(app: AppInfo) -> int:
    if app.active_slot == SLOT_A:
      return SLOT_B
    if app.active_slot == SLOT_B:
      return SLOT_A
    raise BootloaderRefused("application reports an invalid active slot")

  def _wait_for_bootloader(self, timeout: float = 12.0) -> BootInfo:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      info = self.read_bootloader(timeout=0.8)
      if info is not None:
        return info
    raise BootloaderRefused("HRR did not enter its CAN bootloader")

  def _validate_boot_info(self, info: BootInfo, manifest: Manifest, previous: AppInfo | None) -> None:
    if info.product_id != manifest.product_id or info.protocol_version != 1 or info.geometry_error:
      raise BootloaderRefused("bootloader product/protocol/flash geometry mismatch")
    if info.bootloader_version < manifest.required_bootloader_version:
      required = manifest.required_bootloader_version
      raise BootloaderIncompatible(
        f"firmware requires bootloader >= {(required >> 16) & 0xff}.{(required >> 8) & 0xff}.{required & 0xff}; " +
        f"installed {(info.bootloader_version >> 16) & 0xff}.{(info.bootloader_version >> 8) & 0xff}.{info.bootloader_version & 0xff}")
    if manifest.image_size > 0x30000 or manifest.image_size < 8:
      raise CandidateAuthenticationError("candidate image size is outside an application slot")
    if previous is not None:
      expected_target = self._normal_target(previous)
      if info.confirmed_slot != previous.active_slot or manifest.slot != expected_target:
        raise BootloaderRefused("bootloader confirmed slot disagrees with signed candidate/application")
    elif info.confirmed_slot in (SLOT_A, SLOT_B) and manifest.slot == info.confirmed_slot:
      raise BootloaderRefused("recovery candidate would overwrite the confirmed slot")

  def _reconcile_after_timeout(self, token: int, manifest: Manifest, delay: float = 1.1):
    time.sleep(delay)
    status = self.protocol.status()
    if status.session_token not in (0, token):
      raise BootloaderRefused("bootloader reports a different active session")
    if any(status.image_identity_prefix) and status.image_identity_prefix != manifest.image_sha256[:3]:
      raise BootloaderRefused("bootloader session image identity mismatch")
    if status.error:
      raise BootloaderRefused(f"bootloader error {status.error_name}")
    return status

  def _transfer(self, manifest: Manifest, image: bytes, token: int, sequence: int,
                max_chunk: int, *, offset: int = 0) -> tuple[int, int]:
    chunk_size = min(max_chunk, max(8, self.requested_chunk_size))
    chunk_size -= chunk_size % 8
    retries = 0
    started = time.monotonic()
    while offset < len(image):
      self.safety.check_during_update()
      length = min(chunk_size, len(image) - offset)
      if offset + length < len(image):
        length -= length % 8
      chunk = image[offset:offset + length]
      next_sequence = (sequence + 1) & 0xFFFF
      sent_at = time.monotonic()
      try:
        status = self.protocol.data(token, next_sequence, offset, chunk)
      except TransportError:
        retries += 1
        status = self._reconcile_after_timeout(token, manifest)
      if status.error:
        raise BootloaderRefused(f"DATA failed at {offset}: {status.error_name}")
      if status.next_offset == offset + length:
        offset = status.next_offset
        sequence = next_sequence
      elif status.next_offset == offset:
        retries += 1
        if retries > 20:
          raise TransportError("transfer made no durable progress after 20 retries")
        continue
      else:
        raise BootloaderRefused(f"unexpected durable offset {status.next_offset}, expected {offset} or {offset + length}")
      self.reporter.progress(offset, len(image), retries=retries, chunk_size=chunk_size,
                             elapsed=time.monotonic() - started, ack_age=time.monotonic() - sent_at)
    self.reporter.finish_progress()
    return sequence, retries

  def _wait_for_confirmation(self, manifest: Manifest, previous: AppInfo | None,
                             retries: int) -> UpdateResult:
    self.reporter.event(Stage.TRIAL.value, "waiting for trial application and confirmation reset")
    trial_seen = False
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
      self.safety.check_during_update()
      installed = self.read_application(timeout=1.0)
      if installed is None:
        continue
      if installed.product_id != manifest.product_id:
        raise TrialFailure("trial application product identity changed")
      if installed.bootloader_version < manifest.required_bootloader_version:
        raise TrialFailure("trial application reports an incompatible bootloader")
      expected_identity = (installed.active_slot == manifest.slot and
                           installed.build_identity == manifest.build_identity)
      if expected_identity:
        if installed.security_version is not None and installed.security_version != manifest.security_version:
          raise TrialFailure("trial application security version disagrees with the signed manifest")
        if installed.trial:
          trial_seen = True
          continue
        if installed.confirmed and installed.version == manifest.version:
          self.reporter.event(Stage.SUCCESS.value,
                              f"HRR v{installed.version}, slot {SLOT_NAMES[installed.active_slot]} CONFIRMED")
          return UpdateResult(previous, installed, manifest.slot, retries)
      if installed.confirmed and not expected_identity:
        raise RollbackOccurred("Update failed and HRR rolled back safely")
    if trial_seen:
      raise TrialFailure("trial application did not become confirmed")
    raise TrialFailure("candidate application was not observed after reset")

  def flash(self, manifest: Manifest, image: bytes, *, no_reset: bool = False,
            recover: bool = False) -> UpdateResult | None:
    self.reporter.event(Stage.VERIFY_LOCAL.value,
                        f"signed HRR v{manifest.version}, slot {SLOT_NAMES[manifest.slot]}, " +
                        f"security {manifest.security_version}")
    previous, boot = self.discover(query_bootloader=False)
    if (recover and previous is not None and previous.active_slot == manifest.slot and
        previous.build_identity == manifest.build_identity):
      self.reporter.event(Stage.SAFE_STATE.value, "resuming trial/confirmation observation")
      self.safety.acquire()
      release_observation_lease = False
      try:
        self.safety.mark_in_progress()
        result = self._wait_for_confirmation(manifest, previous, 0)
        release_observation_lease = True
        return result
      except RollbackOccurred:
        release_observation_lease = True
        raise
      finally:
        if release_observation_lease:
          self.safety.release()
    if previous is not None:
      if not previous.update_supported or not (previous.flags & FLAG_UPDATE_SUPPORT):
        raise BootloaderRefused("running HRR application does not advertise update support")
      if manifest.slot != self._normal_target(previous):
        raise CandidateAuthenticationError("manifest does not target the inactive application slot")

    self.reporter.event(Stage.SAFE_STATE.value, "checking OFFROAD state")
    self.safety.acquire()
    release_lease = False
    entered_bootloader = False
    committed = False
    token = 0
    try:
      self.safety.check()
      self.safety.mark_in_progress()
      if previous is not None:
        self.reporter.event(Stage.ENTER_BOOTLOADER.value, "requesting safe application reset")
        self.protocol.enter_bootloader()
        boot = self._wait_for_bootloader()
        entered_bootloader = True
      if boot is None:
        boot = self._wait_for_bootloader()
        entered_bootloader = True
      self._validate_boot_info(boot, manifest, previous)
      self.reporter.event(Stage.BOOT_INFO.value,
                          f"bootloader {(boot.bootloader_version >> 16) & 0xff}.{(boot.bootloader_version >> 8) & 0xff}.{boot.bootloader_version & 0xff}; " +
                          f"confirmed slot {SLOT_NAMES.get(boot.confirmed_slot, 'none')}")

      status = self.protocol.status()
      if recover and status.state in (UpdateState.PROGRAMMING, UpdateState.PROGRAMMED, UpdateState.COMMITTED):
        if not status.session_token or status.target_slot != manifest.slot:
          raise BootloaderRefused("bootloader session cannot be matched to this candidate")
        if status.image_identity_prefix != manifest.image_sha256[:3]:
          raise BootloaderRefused("bootloader session belongs to a different image")
        if status.next_offset > len(image):
          raise BootloaderRefused("bootloader durable offset exceeds the candidate image")
        token, sequence = status.session_token, status.sequence
        committed = status.state == UpdateState.COMMITTED
        action = "committed candidate" if committed else f"durable offset {status.next_offset}"
        self.reporter.event(Stage.START_ERASE.value,
                            f"resuming slot {SLOT_NAMES[manifest.slot]} at {action}; session=0x{token:08x}")
      else:
        if status.state not in (UpdateState.IDLE, UpdateState.FAILED):
          raise BootloaderRefused("an update session is already active; use recover with the matching candidate")
        token = secrets.randbits(32) or 1
        sequence = secrets.randbits(16)
        self.reporter.event(Stage.START_ERASE.value,
                            f"erasing slot {SLOT_NAMES[manifest.slot]}; session=0x{token:08x}")
        for start_attempt in range(3):
          try:
            status = self.protocol.start(token, sequence, manifest)
            break
          except TransportError:
            status = self._reconcile_after_timeout(token, manifest, delay=2.0)
            if status.state == UpdateState.IDLE and status.session_token == 0 and start_attempt < 2:
              continue
            sequence = status.sequence
            break
        if status.error or status.state not in (UpdateState.PROGRAMMING, UpdateState.PROGRAMMED):
          raise BootloaderRefused(f"START failed: {status.error_name}, state {status.state.name}")

      retries = 0
      if not committed:
        self.reporter.event(Stage.TRANSFER.value, "uploading with durable acknowledgements")
        sequence, retries = self._transfer(manifest, image, token, sequence, boot.max_chunk,
                                           offset=status.next_offset)
        self.reporter.event(Stage.FINISH.value, "verifying SHA-256/signature and committing TRIAL")
        finish_sequence = (sequence + 1) & 0xFFFF
        for finish_attempt in range(3):
          try:
            status = self.protocol.finish(token, finish_sequence)
            break
          except TransportError:
            status = self._reconcile_after_timeout(token, manifest)
            if status.state == UpdateState.PROGRAMMED and finish_attempt < 2:
              continue
            break
        if status.error or status.state != UpdateState.COMMITTED:
          raise CandidateAuthenticationError(f"FINISH failed: {status.error_name}, state {status.state.name}")
        committed = True
      if no_reset:
        self.reporter.event(Stage.FINISH.value, "candidate committed; reset suppressed by --no-reset")
        return None
      self.protocol.reset()
      try:
        result = self._wait_for_confirmation(manifest, previous, retries)
      except RollbackOccurred:
        release_lease = True
        raise
      release_lease = True
      return result
    except UnsafeStateError:
      if not entered_bootloader:
        release_lease = True
      elif token:
        try:
          self.protocol.abort(token)
          self.protocol.reset()
          if self.read_application(timeout=5.0) is not None:
            release_lease = True
        except Exception:
          pass
      raise
    except Exception:
      if not entered_bootloader:
        release_lease = True
      elif not committed:
        try:
          if token:
            self.protocol.abort(token)
          self.protocol.reset()
          if self.read_application(timeout=5.0) is not None:
            release_lease = True
        except Exception:
          pass
      raise
    finally:
      if release_lease:
        self.safety.release()
