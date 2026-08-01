from __future__ import annotations

import binascii
import hashlib
import io
import struct
import tempfile
from pathlib import Path

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from openpilot.tools.hrr_updater.constants import (APP_INFO0_ID, BOOT_BASE, PRODUCT_ID, SLOT_A, SLOT_B,
                                         SLOT_BASES, SVC_DATA, SVC_FINISH, SVC_START)
from openpilot.tools.hrr_updater.errors import (BootloaderIncompatible, CandidateAuthenticationError,
                                                ImageVerificationError, ReleaseVerificationError,
                                                RollbackOccurred, UnsafeStateError)
from openpilot.tools.hrr_updater.fake import FakeFaults, FakeHrrTransport
from openpilot.tools.hrr_updater.manifest import FULL, PREFIX, Manifest, Version
from openpilot.tools.hrr_updater.output import Reporter
from openpilot.tools.hrr_updater.protocol import AppInfoCollector, HrrProtocol, UpdateState
from openpilot.tools.hrr_updater.release import canonical_json, verify_release_signature
from openpilot.tools.hrr_updater.safety import FakeSafetyGate
from openpilot.tools.hrr_updater.state import StateStore
from openpilot.tools.hrr_updater.transport import CanFrame
from openpilot.tools.hrr_updater.updater import HrrUpdater

SEED = bytes.fromhex("9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60")
PRIVATE = Ed25519PrivateKey.from_private_bytes(SEED)
PUBLIC = bytes.fromhex("d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a")


DEFAULT_ARTIFACT_VERSION = Version(0, 1, 1)


def make_artifacts(slot: int = SLOT_B, version: Version = DEFAULT_ARTIFACT_VERSION,
                   required_bootloader: int = 0x00010100, product: int = PRODUCT_ID,
                   size: int = 520) -> tuple[Manifest, bytes]:
  base = SLOT_BASES[slot]
  image = struct.pack("<II", 0x20020000, base + 0x101) + bytes(index & 0xFF for index in range(size - 8))
  build_id = bytes.fromhex("0123456789abcdef0123456789abcdef01234567")
  prefix = PREFIX.pack(0x4D525248, 1, FULL.size, product, slot, 1, 0, base,
                       base + 0x101, len(image), version.major, version.minor,
                       version.patch, 0, 2, required_bootloader, build_id,
                       hashlib.sha256(image).digest())
  without_crc = prefix + PRIVATE.sign(prefix)
  raw = without_crc + struct.pack("<I", binascii.crc32(without_crc) & 0xFFFFFFFF)
  return Manifest.parse(raw), image


def updater(transport: FakeHrrTransport, safety: FakeSafetyGate | None = None) -> HrrUpdater:
  return HrrUpdater(transport, safety or FakeSafetyGate(), transport.bus,
                    Reporter(plain=True, stream=io.StringIO()), chunk_size=128, frame_pacing=0)


class OnroadAfterLeaseGate(FakeSafetyGate):
  def acquire(self, timeout: float = 10.0) -> None:
    super().acquire(timeout)
    self.safe = False


class Assertions:
  def assertEqual(self, actual: object, expected: object) -> None:
    assert actual == expected

  def assertTrue(self, value: object) -> None:
    assert value

  def assertFalse(self, value: object) -> None:
    assert not value

  def assertIsNone(self, value: object) -> None:
    assert value is None

  def assertGreaterEqual(self, actual: int, expected: int) -> None:
    assert actual >= expected


class TestManifestAndRelease(Assertions):
  def test_manifest_and_image_verify(self):
    manifest, image = make_artifacts()
    manifest.verify(image, PUBLIC)
    self.assertEqual(manifest.slot, SLOT_B)

  def test_hash_signature_and_wrong_product_fail_closed(self):
    manifest, image = make_artifacts()
    with self.assertRaises(ImageVerificationError):
      manifest.verify(image[:-1] + b"x", PUBLIC)
    with self.assertRaises(ImageVerificationError):
      manifest.verify(image, bytes(32))
    with self.assertRaises(ImageVerificationError):
      make_artifacts(product=PRODUCT_ID + 1)

  def test_bootloader_range_is_refused(self):
    manifest, _ = make_artifacts()
    values = list(FULL.unpack(manifest.raw))
    values[7] = BOOT_BASE
    raw = FULL.pack(*values)
    raw = raw[:-4] + struct.pack("<I", binascii.crc32(raw[:-4]) & 0xFFFFFFFF)
    with self.assertRaises(ImageVerificationError):
      Manifest.parse(raw)

  def test_release_json_canonical_signature(self):
    slot = {"image": "a.bin", "manifest": "a.manifest", "size": 8,
            "sha256": "00", "manifest_sha256": "00"}
    raw = canonical_json({"schema_version": 1, "product": "HRR", "product_id": PRODUCT_ID,
                          "hardware_id": "HRR_G474_V1", "required_bootloader_version": 0x10100,
                          "minimum_updater_version": "1.0.0",
                          "slots": {"A": slot, "B": slot}})
    parsed = verify_release_signature(raw, PRIVATE.sign(raw), PUBLIC)
    self.assertEqual(parsed["product_id"], PRODUCT_ID)
    with pytest.raises(ReleaseVerificationError):
      verify_release_signature(raw + b" ", PRIVATE.sign(raw), PUBLIC)


class TestProtocolAndDiscovery(Assertions):
  def test_application_discovery_filters_bus_and_id(self):
    collector = AppInfoCollector(2)
    self.assertIsNone(collector.add(CanFrame(APP_INFO0_ID, bytes(8), 1)))
    self.assertIsNone(collector.add(CanFrame(0x100, bytes(8), 2)))
    transport = FakeHrrTransport()
    info = updater(transport).read_application()
    self.assertEqual(info.active_slot, SLOT_A)
    self.assertTrue(info.confirmed)

  def test_bootloader_recovery_discovery(self):
    transport = FakeHrrTransport(recovery=True, active_slot=SLOT_A)
    info = updater(transport).read_bootloader()
    self.assertEqual(info.product_id, PRODUCT_ID)
    self.assertEqual(info.max_chunk, 512)

  def test_duplicate_start_and_data_are_idempotent(self):
    manifest, image = make_artifacts()
    transport = FakeHrrTransport(recovery=True)
    protocol = HrrProtocol(transport, 2, frame_pacing=0)
    first = protocol.start(7, 10, manifest)
    repeated = protocol.start(7, 10, manifest)
    self.assertEqual(first.next_offset, repeated.next_offset)
    accepted = protocol.data(7, 11, 0, image[:128])
    duplicate = protocol.data(7, 11, 0, image[:128])
    self.assertEqual(accepted.next_offset, duplicate.next_offset)

  def test_sequence_mismatch_reported(self):
    manifest, image = make_artifacts()
    transport = FakeHrrTransport(recovery=True)
    protocol = HrrProtocol(transport, 2, frame_pacing=0)
    protocol.start(7, 10, manifest)
    status = protocol.data(7, 13, 0, image[:128])
    self.assertEqual(status.error_name, "SEQUENCE")


class TestUpdateStateMachine(Assertions):
  def test_active_a_selects_and_confirms_b(self):
    manifest, image = make_artifacts(SLOT_B)
    result = updater(FakeHrrTransport(active_slot=SLOT_A)).flash(manifest, image)
    self.assertEqual(result.target_slot, SLOT_B)
    self.assertTrue(result.installed.confirmed)
    self.assertEqual(result.installed.version, manifest.version)

  def test_active_b_selects_and_confirms_a(self):
    manifest, image = make_artifacts(SLOT_A)
    result = updater(FakeHrrTransport(active_slot=SLOT_B)).flash(manifest, image)
    self.assertEqual(result.target_slot, SLOT_A)

  def test_wrong_active_slot_candidate_rejected_before_entry(self):
    manifest, image = make_artifacts(SLOT_A)
    transport = FakeHrrTransport(active_slot=SLOT_A)
    with self.assertRaises(CandidateAuthenticationError):
      updater(transport).flash(manifest, image)
    self.assertEqual(transport.mode, "app")

  def test_incompatible_bootloader_refused(self):
    manifest, image = make_artifacts(required_bootloader=0x00010200)
    with self.assertRaises(BootloaderIncompatible):
      updater(FakeHrrTransport()).flash(manifest, image)

  def test_unsafe_host_refused_before_bootloader(self):
    manifest, image = make_artifacts()
    transport = FakeHrrTransport()
    with self.assertRaises(UnsafeStateError):
      updater(transport, FakeSafetyGate(safe=False)).flash(manifest, image)
    self.assertEqual(transport.mode, "app")

  def test_onroad_transition_after_lease_aborts_before_entry(self):
    manifest, image = make_artifacts()
    transport = FakeHrrTransport()
    gate = OnroadAfterLeaseGate()
    with self.assertRaises(UnsafeStateError):
      updater(transport, gate).flash(manifest, image)
    self.assertEqual(transport.mode, "app")
    self.assertFalse(gate.acquired)

  def test_ignition_during_transfer_aborts_and_restores_application(self):
    manifest, image = make_artifacts()
    transport = FakeHrrTransport()
    gate = FakeSafetyGate(ignition=True)
    with self.assertRaises(UnsafeStateError):
      updater(transport, gate).flash(manifest, image)
    self.assertEqual(transport.mode, "app")
    self.assertFalse(gate.in_progress)

  def test_ignition_powered_pre_onroad_update_is_allowed_by_startup_hold(self):
    manifest, image = make_artifacts()
    gate = FakeSafetyGate(ignition=True, allow_ignition=True)
    result = updater(FakeHrrTransport(), gate).flash(manifest, image)
    self.assertTrue(result.installed.confirmed)
    self.assertFalse(gate.in_progress)

  def test_lost_start_data_and_finish_acks_reconcile_durable_state(self, monkeypatch: pytest.MonkeyPatch):
    manifest, image = make_artifacts()
    faults = FakeFaults()
    faults.lose_ack(SVC_START)
    faults.lose_ack(SVC_DATA)
    faults.lose_ack(SVC_FINISH)
    monkeypatch.setattr("openpilot.tools.hrr_updater.updater.time.sleep", lambda _: None)
    result = updater(FakeHrrTransport(faults=faults)).flash(manifest, image)
    assert result is not None
    self.assertTrue(result.installed.confirmed)
    self.assertGreaterEqual(result.retries, 1)

  def test_lost_start_data_and_finish_requests_are_retried(self, monkeypatch: pytest.MonkeyPatch):
    manifest, image = make_artifacts()
    faults = FakeFaults()
    faults.lose_request(SVC_START)
    faults.lose_request(SVC_DATA)
    faults.lose_request(SVC_FINISH)
    monkeypatch.setattr("openpilot.tools.hrr_updater.updater.time.sleep", lambda _: None)
    result = updater(FakeHrrTransport(faults=faults)).flash(manifest, image)
    assert result is not None
    self.assertTrue(result.installed.confirmed)

  def test_host_restart_resumes_durable_offset(self):
    manifest, image = make_artifacts()
    transport = FakeHrrTransport(recovery=True, active_slot=SLOT_A)
    protocol = HrrProtocol(transport, 2, frame_pacing=0)
    protocol.start(123, 9, manifest)
    protocol.data(123, 10, 0, image[:128])
    result = updater(transport).flash(manifest, image, recover=True)
    self.assertEqual(result.installed.active_slot, SLOT_B)

  def test_host_restart_after_commit_resets_and_observes_trial(self):
    manifest, image = make_artifacts()
    transport = FakeHrrTransport(recovery=True, active_slot=SLOT_A)
    protocol = HrrProtocol(transport, 2, frame_pacing=0)
    protocol.start(123, 9, manifest)
    status = protocol.data(123, 10, 0, image[:512])
    status = protocol.data(123, 11, 512, image[512:])
    self.assertEqual(status.state, UpdateState.PROGRAMMED)
    self.assertEqual(protocol.finish(123, 12).state, UpdateState.COMMITTED)
    result = updater(transport).flash(manifest, image, recover=True)
    self.assertTrue(result.installed.confirmed)

  def test_host_restart_while_trial_is_running_only_observes(self):
    manifest, image = make_artifacts()
    transport = FakeHrrTransport(active_slot=SLOT_B, version=manifest.version)
    transport.build_identity = manifest.build_identity
    transport.security_version = manifest.security_version
    transport.rx.clear()
    transport._queue_app(trial=True)
    result = updater(transport).flash(manifest, image, recover=True)
    self.assertTrue(result.installed.confirmed)

  def test_recovery_without_confirmed_application(self):
    manifest, image = make_artifacts(SLOT_A)
    transport = FakeHrrTransport(recovery=True, active_slot=0)
    result = updater(transport).flash(manifest, image)
    self.assertEqual(result.installed.active_slot, SLOT_A)

  def test_candidate_authentication_failure(self):
    manifest, image = make_artifacts()
    faults = FakeFaults(verification_error=11)
    with pytest.raises(CandidateAuthenticationError):
      updater(FakeHrrTransport(faults=faults)).flash(manifest, image)

  def test_trial_rollback_is_detected_and_lease_released(self):
    manifest, image = make_artifacts()
    gate = FakeSafetyGate()
    with self.assertRaises(RollbackOccurred):
      updater(FakeHrrTransport(faults=FakeFaults(rollback=True)), gate).flash(manifest, image)
    self.assertFalse(gate.in_progress)


class TestPersistentPolicy(Assertions):
  def test_failed_candidate_suppression_identity_is_exact(self):
    with tempfile.TemporaryDirectory() as directory:
      store = StateStore(Path(directory) / "state.json")
      store.record_failed("0.1.1", "abc", "rollback")
      self.assertTrue(store.is_failed("0.1.1", "abc"))
      self.assertFalse(store.is_failed("0.1.2", "abc"))
      self.assertFalse(store.is_failed("0.1.1", "def"))

  def test_active_candidate_survives_restart_and_clears(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "state.json"
      store = StateStore(path)
      store.set_active(Path("a.manifest"), Path("a.bin"), "0.1.1", "abc")
      self.assertEqual(StateStore(path).active()["image_identity"], "abc")
      store.clear_active()
      self.assertIsNone(store.active())

  def test_complete_ab_candidate_is_staged_before_ignition(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "state.json"
      store = StateStore(path)
      slots = {
        "A": {"manifest": "a.manifest", "image": "a.bin", "image_identity": "aaa"},
        "B": {"manifest": "b.manifest", "image": "b.bin", "image_identity": "bbb"},
      }
      store.set_pending("v0.2.0", "0.2.0", slots)
      pending = StateStore(path).pending()
      self.assertEqual(set(pending["slots"]), {"A", "B"})
      self.assertEqual(pending["slots"]["B"]["image_identity"], "bbb")
      store.set_installed_version("0.1.1")
      self.assertEqual(store.installed_version(), "0.1.1")
      store.clear_pending()
      self.assertIsNone(store.pending())
