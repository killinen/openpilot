from __future__ import annotations

import binascii
import hashlib
import io
import struct
import tempfile
from collections import deque
from pathlib import Path

import pytest
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from openpilot.tools.trqi_updater.cli import build_parser
from openpilot.tools.trqi_updater.constants import (APP_INFO0_ID, BOOT_BASE, DEFAULT_FRAME_PACING, PRODUCT_ID,
                                         SLOT_A, SLOT_B, SLOT_BASES, SVC_DATA, SVC_FINISH, SVC_START)
from openpilot.tools.trqi_updater.errors import (BootloaderIncompatible, CandidateAuthenticationError,
                                                ImageVerificationError, ReleaseVerificationError, BootloaderRefused,
                                                RollbackOccurred, TransportError, UnsafeStateError)
from openpilot.tools.trqi_updater.fake import FakeFaults, FakeTrqiTransport
from openpilot.tools.trqi_updater.manifest import FULL, PREFIX, Manifest, Version
from openpilot.tools.trqi_updater.openpilot_transport import OpenpilotPandaTransport
from openpilot.tools.trqi_updater.output import Reporter
from openpilot.tools.trqi_updater.protocol import AppInfoCollector, TrqiProtocol, UpdateState
from openpilot.tools.trqi_updater.release import canonical_json, verify_release_signature
from openpilot.tools.trqi_updater.safety import FakeSafetyGate, ignition_interlock_active
from openpilot.tools.trqi_updater.state import StateStore
from openpilot.tools.trqi_updater.transport import CanFrame
from openpilot.tools.trqi_updater.updater import TrqiUpdater

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
  prefix = PREFIX.pack(0x4D515254, 1, FULL.size, product, slot, 1, 0, base,
                       base + 0x101, len(image), version.major, version.minor,
                       version.patch, 0, 2, required_bootloader, build_id,
                       hashlib.sha256(image).digest())
  without_crc = prefix + PRIVATE.sign(prefix)
  raw = without_crc + struct.pack("<I", binascii.crc32(without_crc) & 0xFFFFFFFF)
  return Manifest.parse(raw), image


def updater(transport: FakeTrqiTransport, safety: FakeSafetyGate | None = None) -> TrqiUpdater:
  return TrqiUpdater(transport, safety or FakeSafetyGate(), transport.bus,
                    Reporter(plain=True, stream=io.StringIO()), chunk_size=128, frame_pacing=0)


class OnroadAfterLeaseGate(FakeSafetyGate):
  def acquire(self, timeout: float = 10.0) -> None:
    super().acquire(timeout)
    self.safe = False


class FakeSendcanPublisher:
  def __init__(self, events: list[tuple], ready: bool):
    self.events = events
    self.ready = ready

  def wait_for_readers_to_update(self, service: str, timeout: int) -> bool:
    self.events.append(("wait", service, timeout))
    return self.ready

  def send(self, service: str, message: object) -> None:
    self.events.append(("send", service, message))


class FakeSendcanMessaging:
  def __init__(self, events: list[tuple], readiness: list[bool]):
    self.events = events
    self.readiness = deque(readiness)

  def PubMaster(self, services: list[str]) -> FakeSendcanPublisher:
    self.events.append(("create", tuple(services)))
    return FakeSendcanPublisher(self.events, self.readiness.popleft())


def openpilot_transport(events: list[tuple], readiness: list[bool], receipts: list[CanFrame]) -> OpenpilotPandaTransport:
  transport = OpenpilotPandaTransport.__new__(OpenpilotPandaTransport)
  transport.messaging = FakeSendcanMessaging(events, readiness)
  transport.can_list_to_can_capnp = lambda messages, msgtype: (messages, msgtype)
  transport.sendcan = None
  transport.receipt_timeout = 0.05
  transport.pending = deque()
  transport.closed = False
  receipt_queue = deque(receipts)
  transport._receive_event = lambda timeout: [receipt_queue.popleft()] if receipt_queue else []
  return transport


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

  def assertRaises(self, exception):
    return pytest.raises(exception)


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
            "sha256": "00" * 32, "manifest_sha256": "00" * 32}
    raw = canonical_json({"schema_version": 1, "product": "TRQI", "product_id": PRODUCT_ID,
                          "hardware_id": "TRQI_G474_V1", "required_bootloader_version": 0x10100,
                          "minimum_updater_version": "1.0.0", "firmware_version": "0.2.0",
                          "security_version": 1, "git_sha": "1a78aab70267d53ccaf0fd40652a74a9e3f4f4f0",
                          "slots": {"A": slot, "B": slot}})
    parsed = verify_release_signature(raw, PRIVATE.sign(raw), PUBLIC)
    self.assertEqual(parsed["product_id"], PRODUCT_ID)
    with pytest.raises(ReleaseVerificationError):
      verify_release_signature(raw + b" ", PRIVATE.sign(raw), PUBLIC)

  def test_release_metadata_rejects_unsafe_asset_name(self):
    slot = {"image": "../a.bin", "manifest": "a.manifest", "size": 8,
            "sha256": "00" * 32, "manifest_sha256": "00" * 32}
    raw = canonical_json({"schema_version": 1, "product": "TRQI", "product_id": PRODUCT_ID,
                          "hardware_id": "TRQI_G474_V1", "required_bootloader_version": 0x10100,
                          "minimum_updater_version": "1.0.0", "firmware_version": "0.2.0",
                          "security_version": 1, "git_sha": "1a78aab70267d53ccaf0fd40652a74a9e3f4f4f0",
                          "slots": {"A": slot, "B": slot}})
    with pytest.raises(ReleaseVerificationError):
      verify_release_signature(raw, PRIVATE.sign(raw), PUBLIC)


class TestProtocolAndDiscovery(Assertions):
  def test_shared_default_frame_pacing_is_five_milliseconds(self):
    transport = FakeTrqiTransport()
    protocol = TrqiProtocol(transport, 1)
    updater_instance = TrqiUpdater(transport, FakeSafetyGate(), 1)
    args = build_parser().parse_args(["info"])

    self.assertEqual(DEFAULT_FRAME_PACING, 0.005)
    self.assertEqual(protocol.isotp.frame_pacing, DEFAULT_FRAME_PACING)
    self.assertEqual(updater_instance.protocol.isotp.frame_pacing, DEFAULT_FRAME_PACING)
    self.assertEqual(args.frame_pacing, DEFAULT_FRAME_PACING)

  def test_application_discovery_filters_bus_and_id(self):
    collector = AppInfoCollector(1)
    self.assertIsNone(collector.add(CanFrame(APP_INFO0_ID, bytes(8), 2)))
    self.assertIsNone(collector.add(CanFrame(0x100, bytes(8), 1)))
    transport = FakeTrqiTransport()
    info = updater(transport).read_application()
    self.assertEqual(info.active_slot, SLOT_A)
    self.assertTrue(info.confirmed)

  def test_bootloader_recovery_discovery(self):
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    info = updater(transport).read_bootloader()
    self.assertEqual(info.product_id, PRODUCT_ID)
    self.assertEqual(info.max_chunk, 512)

  def test_duplicate_start_and_data_are_idempotent(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport(recovery=True)
    protocol = TrqiProtocol(transport, 1, frame_pacing=0)
    first = protocol.start(7, 10, manifest)
    repeated = protocol.start(7, 10, manifest)
    self.assertEqual(first.next_offset, repeated.next_offset)
    accepted = protocol.data(7, 11, 0, image[:128])
    duplicate = protocol.data(7, 11, 0, image[:128])
    self.assertEqual(accepted.next_offset, duplicate.next_offset)

  def test_sequence_mismatch_reported(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport(recovery=True)
    protocol = TrqiProtocol(transport, 1, frame_pacing=0)
    protocol.start(7, 10, manifest)
    status = protocol.data(7, 13, 0, image[:128])
    self.assertEqual(status.error_name, "SEQUENCE")


class TestOpenpilotTransport(Assertions):
  def test_first_send_waits_for_pandad_reader_before_publish(self):
    events: list[tuple] = []
    data = bytes.fromhex("b00178563412a5bb")
    transport = openpilot_transport(events, [True], [CanFrame(0x60A, data, 0x81)])

    self.assertEqual(events, [])
    transport.send(0x60A, data, 1)

    self.assertEqual(events[0], ("create", ("sendcan",)))
    self.assertEqual(events[1], ("wait", "sendcan", 2))
    self.assertEqual(events[2][0], "send")

  def test_tx_receipt_does_not_discard_later_response_in_same_batch(self):
    events: list[tuple] = []
    request = bytes.fromhex("0222000000000000")
    response = CanFrame(0x6A1, bytes.fromhex("0762010100010000"), 1)
    transport = openpilot_transport(events, [True], [])
    transport._receive_event = lambda timeout: [CanFrame(0x6A0, request, 0x81), response]

    transport.send(0x6A0, request, 1)

    self.assertEqual(transport.recv(0), response)

  def test_unready_publisher_sends_nothing_and_can_be_reacquired(self):
    events: list[tuple] = []
    data = bytes.fromhex("b00178563412a5bb")
    transport = openpilot_transport(events, [False, True], [CanFrame(0x60A, data, 0x81)])

    with self.assertRaises(TransportError):
      transport.send(0x60A, data, 1)
    self.assertFalse(any(event[0] == "send" for event in events))
    self.assertIsNone(transport.sendcan)

    transport.send(0x60A, data, 1)
    self.assertEqual(sum(event[0] == "create" for event in events), 2)
    self.assertEqual(sum(event[0] == "send" for event in events), 1)

  def test_safety_rejected_receipt_remains_a_hard_error(self):
    events: list[tuple] = []
    data = bytes.fromhex("b00178563412a5bb")
    transport = openpilot_transport(events, [True], [CanFrame(0x60A, data, 0xC1)])

    with pytest.raises(TransportError, match="Panda safety rejected"):
      transport.send(0x60A, data, 1)

  def test_safety_rejection_takes_precedence_over_success_in_same_batch(self):
    events: list[tuple] = []
    data = bytes.fromhex("b00178563412a5bb")
    transport = openpilot_transport(events, [True], [])
    transport._receive_event = lambda timeout: [
      CanFrame(0x60A, data, 0x81),
      CanFrame(0x60A, data, 0xC1),
    ]

    with pytest.raises(TransportError, match="Panda safety rejected"):
      transport.send(0x60A, data, 1)


class TestUpdateStateMachine(Assertions):
  def test_persisted_update_keeps_ignition_interlock_after_manager_restart(self):
    self.assertTrue(ignition_interlock_active(False, True))
    self.assertTrue(ignition_interlock_active(True, False))
    self.assertFalse(ignition_interlock_active(False, False))

  def test_active_a_selects_and_confirms_b(self):
    manifest, image = make_artifacts(SLOT_B)
    result = updater(FakeTrqiTransport(active_slot=SLOT_A)).flash(manifest, image)
    self.assertEqual(result.target_slot, SLOT_B)
    self.assertTrue(result.installed.confirmed)
    self.assertEqual(result.installed.version, manifest.version)

  def test_active_b_selects_and_confirms_a(self):
    manifest, image = make_artifacts(SLOT_A)
    result = updater(FakeTrqiTransport(active_slot=SLOT_B)).flash(manifest, image)
    self.assertEqual(result.target_slot, SLOT_A)

  def test_wrong_active_slot_candidate_rejected_before_entry(self):
    manifest, image = make_artifacts(SLOT_A)
    transport = FakeTrqiTransport(active_slot=SLOT_A)
    with self.assertRaises(CandidateAuthenticationError):
      updater(transport).flash(manifest, image)
    self.assertEqual(transport.mode, "app")

  def test_incompatible_bootloader_refused(self):
    manifest, image = make_artifacts(required_bootloader=0x00010200)
    with self.assertRaises(BootloaderIncompatible):
      updater(FakeTrqiTransport()).flash(manifest, image)

  def test_unsafe_host_refused_before_bootloader(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport()
    with self.assertRaises(UnsafeStateError):
      updater(transport, FakeSafetyGate(safe=False)).flash(manifest, image)
    self.assertEqual(transport.mode, "app")

  def test_onroad_transition_after_lease_aborts_before_entry(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport()
    gate = OnroadAfterLeaseGate()
    with self.assertRaises(UnsafeStateError):
      updater(transport, gate).flash(manifest, image)
    self.assertEqual(transport.mode, "app")
    self.assertFalse(gate.acquired)

  def test_ignition_during_transfer_aborts_and_restores_application(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport()
    gate = FakeSafetyGate(ignition=True)
    with self.assertRaises(UnsafeStateError):
      updater(transport, gate).flash(manifest, image)
    self.assertEqual(transport.mode, "app")
    self.assertFalse(gate.in_progress)

  def test_ignition_powered_pre_onroad_update_is_allowed_by_startup_hold(self):
    manifest, image = make_artifacts()
    gate = FakeSafetyGate(ignition=True, allow_ignition=True)
    result = updater(FakeTrqiTransport(), gate).flash(manifest, image)
    self.assertTrue(result.installed.confirmed)
    self.assertFalse(gate.in_progress)

  def test_lost_start_data_and_finish_acks_reconcile_durable_state(self, monkeypatch: pytest.MonkeyPatch):
    manifest, image = make_artifacts()
    faults = FakeFaults()
    faults.lose_ack(SVC_START)
    faults.lose_ack(SVC_DATA)
    faults.lose_ack(SVC_FINISH)
    monkeypatch.setattr("openpilot.tools.trqi_updater.updater.time.sleep", lambda _: None)
    result = updater(FakeTrqiTransport(faults=faults)).flash(manifest, image)
    assert result is not None
    self.assertTrue(result.installed.confirmed)
    self.assertGreaterEqual(result.retries, 1)
    self.assertGreaterEqual(result.timeouts, 3)

  def test_lost_start_data_and_finish_requests_are_retried(self, monkeypatch: pytest.MonkeyPatch):
    manifest, image = make_artifacts()
    faults = FakeFaults()
    faults.lose_request(SVC_START)
    faults.lose_request(SVC_DATA)
    faults.lose_request(SVC_FINISH)
    monkeypatch.setattr("openpilot.tools.trqi_updater.updater.time.sleep", lambda _: None)
    result = updater(FakeTrqiTransport(faults=faults)).flash(manifest, image)
    assert result is not None
    self.assertTrue(result.installed.confirmed)

  def test_host_restart_resumes_durable_offset(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    protocol = TrqiProtocol(transport, 1, frame_pacing=0)
    protocol.start(123, 9, manifest)
    protocol.data(123, 10, 0, image[:128])
    result = updater(transport).flash(manifest, image, recover=True)
    self.assertEqual(result.installed.active_slot, SLOT_B)

  def test_normal_upload_resumes_matching_session(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    protocol = TrqiProtocol(transport, 1, frame_pacing=0)
    protocol.start(123, 9, manifest)
    protocol.data(123, 10, 0, image[:128])
    result = updater(transport).flash(manifest, image)
    self.assertTrue(result.installed.confirmed)
    self.assertEqual(transport.token, 123)

  def test_normal_upload_replaces_old_inactive_session(self):
    old, _ = make_artifacts(size=75612)
    manifest, image = make_artifacts(size=75616)
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    protocol = TrqiProtocol(transport, 1, frame_pacing=0)
    protocol.start(123, 9, old)
    transport.offset = 65152  # Old 1.1.1 journal-full failure on car board.
    transport.mode = "app"
    transport.rx.clear()
    result = updater(transport).flash(manifest, image)
    self.assertTrue(result.installed.confirmed)
    assert transport.token != 123
    self.assertEqual(transport.manifest.raw, manifest.raw)

  def test_unobserved_fallback_does_not_allow_discard(self):
    old, _ = make_artifacts(size=520)
    manifest, image = make_artifacts(size=528)
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    TrqiProtocol(transport, 1, frame_pacing=0).start(123, 9, old)
    with pytest.raises(BootloaderRefused, match="cannot safely replace"):
      updater(transport).flash(manifest, image)
    self.assertEqual(transport.token, 123)
    self.assertEqual(transport.manifest.raw, old.raw)

  def test_short_hash_collision_does_not_resume_or_erase(self):
    old, _ = make_artifacts(size=520)
    manifest, image = make_artifacts(size=528)
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    TrqiProtocol(transport, 1, frame_pacing=0).start(123, 9, old)
    transport.image_identity = manifest.image_sha256[:3]
    with pytest.raises(BootloaderRefused, match="complete signed manifest"):
      updater(transport).flash(manifest, image)
    self.assertEqual(transport.manifest.raw, old.raw)

  def test_failed_abort_does_not_start_new_image(self, monkeypatch):
    old, _ = make_artifacts(size=520)
    manifest, image = make_artifacts(size=528)
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    TrqiProtocol(transport, 1, frame_pacing=0).start(123, 9, old)
    transport.mode = "app"
    transport.rx.clear()
    runner = updater(transport)
    monkeypatch.setattr(runner.protocol, "abort", lambda token: runner.protocol.status())
    with pytest.raises(BootloaderRefused, match="abort was not confirmed"):
      runner.flash(manifest, image)
    self.assertEqual(transport.manifest.raw, old.raw)

  def test_committed_other_image_is_not_aborted(self):
    old, old_image = make_artifacts(size=520)
    manifest, image = make_artifacts(size=528)
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    protocol = TrqiProtocol(transport, 1, frame_pacing=0)
    protocol.start(123, 9, old)
    protocol.data(123, 10, 0, old_image[:512])
    protocol.data(123, 11, 512, old_image[512:])
    protocol.finish(123, 12)
    with pytest.raises(BootloaderRefused, match="cannot safely replace"):
      updater(transport).flash(manifest, image)
    self.assertEqual(transport.manifest.raw, old.raw)

  def test_session_targeting_confirmed_slot_is_not_aborted(self):
    old, _ = make_artifacts(size=520)
    manifest, image = make_artifacts(size=528)
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    TrqiProtocol(transport, 1, frame_pacing=0).start(123, 9, old)
    transport.target_slot = SLOT_A
    transport.mode = "app"
    transport.rx.clear()
    with pytest.raises(BootloaderRefused, match="cannot safely replace"):
      updater(transport).flash(manifest, image)
    self.assertEqual(transport.token, 123)

  def test_update_result_survives_onroad_transition(self):
    source = (Path(__file__).resolve().parents[1] / "common/params.cc").read_text()
    for key in ("TrqiUpdateProgress", "TrqiUpdateStatus"):
      line = next(line for line in source.splitlines() if '"' + key + '"' in line)
      assert "CLEAR_ON_ONROAD_TRANSITION" not in line
      assert "CLEAR_ON_MANAGER_START" in line

  def test_host_restart_after_commit_resets_and_observes_trial(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport(recovery=True, active_slot=SLOT_A)
    protocol = TrqiProtocol(transport, 1, frame_pacing=0)
    protocol.start(123, 9, manifest)
    status = protocol.data(123, 10, 0, image[:512])
    status = protocol.data(123, 11, 512, image[512:])
    self.assertEqual(status.state, UpdateState.PROGRAMMED)
    self.assertEqual(protocol.finish(123, 12).state, UpdateState.COMMITTED)
    result = updater(transport).flash(manifest, image, recover=True)
    self.assertTrue(result.installed.confirmed)

  def test_host_restart_while_trial_is_running_only_observes(self):
    manifest, image = make_artifacts()
    transport = FakeTrqiTransport(active_slot=SLOT_B, version=manifest.version)
    transport.build_identity = manifest.build_identity
    transport.security_version = manifest.security_version
    transport.rx.clear()
    transport._queue_app(trial=True)
    result = updater(transport).flash(manifest, image, recover=True)
    self.assertTrue(result.installed.confirmed)

  def test_recovery_without_confirmed_application(self):
    manifest, image = make_artifacts(SLOT_A)
    transport = FakeTrqiTransport(recovery=True, active_slot=0)
    result = updater(transport).flash(manifest, image)
    self.assertEqual(result.installed.active_slot, SLOT_A)

  def test_candidate_authentication_failure(self):
    manifest, image = make_artifacts()
    faults = FakeFaults(verification_error=11)
    with pytest.raises(CandidateAuthenticationError):
      updater(FakeTrqiTransport(faults=faults)).flash(manifest, image)

  def test_trial_rollback_is_detected_and_lease_released(self):
    manifest, image = make_artifacts()
    gate = FakeSafetyGate()
    with self.assertRaises(RollbackOccurred):
      updater(FakeTrqiTransport(faults=FakeFaults(rollback=True)), gate).flash(manifest, image)
    self.assertFalse(gate.in_progress)


class TestPersistentPolicy(Assertions):
  def test_failed_candidate_suppression_identity_is_exact(self):
    with tempfile.TemporaryDirectory() as directory:
      store = StateStore(Path(directory) / "state.json")
      store.set_active(Path("a.manifest"), Path("a.bin"), "0.1.1", "abc")
      store.record_failed("0.1.1", "abc", "rollback")
      self.assertTrue(store.is_failed("0.1.1", "abc"))
      self.assertFalse(store.is_failed("0.1.2", "abc"))
      self.assertFalse(store.is_failed("0.1.1", "def"))
      self.assertIsNone(store.active())

  def test_active_candidate_survives_restart_and_clears(self):
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / "state.json"
      store = StateStore(path)
      store.set_active(Path("a.manifest"), Path("a.bin"), "0.1.1", "abc", trust_profile="test")
      self.assertEqual(StateStore(path).active()["image_identity"], "abc")
      self.assertEqual(StateStore(path).active()["trust_profile"], "test")
      store.clear_active()
      self.assertIsNone(store.active())

  def test_stale_active_candidate_matching_failure_is_reconciled(self):
    with tempfile.TemporaryDirectory() as directory:
      store = StateStore(Path(directory) / "state.json")
      store.save({
        "active_candidate": {"version": "0.1.1", "image_identity": "abc"},
        "failed_candidate": {"version": "0.1.1", "image_identity": "abc"},
      })
      self.assertTrue(store.clear_stale_active())
      self.assertIsNone(store.active())

  def test_active_candidate_for_another_identity_is_preserved(self):
    with tempfile.TemporaryDirectory() as directory:
      store = StateStore(Path(directory) / "state.json")
      store.save({
        "active_candidate": {"version": "0.1.2", "image_identity": "def"},
        "failed_candidate": {"version": "0.1.1", "image_identity": "abc"},
      })
      self.assertFalse(store.clear_stale_active())
      self.assertEqual(store.active()["image_identity"], "def")

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
      self.assertEqual(pending["trust_profile"], "production")
      self.assertFalse(pending["force_install"])
      store.set_installed_version("0.1.1")
      self.assertEqual(store.installed_version(), "0.1.1")
      store.clear_pending()
      self.assertIsNone(store.pending())

  def test_test_candidate_is_explicit_and_force_installable(self):
    with tempfile.TemporaryDirectory() as directory:
      store = StateStore(Path(directory) / "state.json")
      slots = {"A": {"manifest": "a", "image": "a", "image_identity": "a"},
               "B": {"manifest": "b", "image": "b", "image_identity": "b"}}
      store.set_pending("local-1a78aab", "0.2.0", slots, trust_profile="test", force_install=True)
      pending = store.pending()
      self.assertEqual(pending["trust_profile"], "test")
      self.assertTrue(pending["force_install"])
