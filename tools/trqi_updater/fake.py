from __future__ import annotations

import binascii
import struct
from collections import defaultdict, deque
from dataclasses import dataclass, field

from openpilot.tools.trqi_updater.constants import (APP_INFO0_ID, APP_INFO1_ID, APP_INFO2_ID, APP_INFO3_ID,
                                         BOOT_REQUEST_ID, BOOT_RESPONSE_ID, ENTER_BOOT_ID,
                                         FLAG_CONFIRMED, FLAG_TRIAL, FLAG_UPDATE_SUPPORT, PRODUCT_ID,
                                         SLOT_A, SLOT_NONE, SVC_ABORT, SVC_DATA, SVC_FINISH,
                                         SVC_GET_INFO, SVC_RESET, SVC_START, SVC_STATUS)
from openpilot.tools.trqi_updater.manifest import Manifest, Version
from openpilot.tools.trqi_updater.protocol import UpdateState
from openpilot.tools.trqi_updater.transport import CanFrame, CanTransport


@dataclass
class FakeFaults:
  corrupt_next_response: dict[int, int] = field(default_factory=lambda: defaultdict(int))
  reject_next_request: dict[int, int] = field(default_factory=lambda: defaultdict(int))
  rollback: bool = False
  verification_error: int = 0

  def lose_ack(self, service: int, count: int = 1) -> None:
    self.corrupt_next_response[service] += count

  def lose_request(self, service: int, count: int = 1) -> None:
    self.reject_next_request[service] += count


class FakeTrqiTransport(CanTransport):
  """In-memory application/bootloader with durable state and ACK fault injection."""

  def __init__(self, bus: int = 1, *, active_slot: int = SLOT_A,
               version: Version | None = None, bootloader_version: int = 0x00010100,
               recovery: bool = False, faults: FakeFaults | None = None):
    self.bus = bus
    self.mode = "boot" if recovery else "app"
    self.active_slot = active_slot
    self.confirmed_slot = SLOT_NONE if recovery and active_slot == SLOT_NONE else active_slot
    self.version = version or Version(0, 1, 0)
    self.build_identity = 0x01020304
    self.bootloader_version = bootloader_version
    self.security_version = 1
    self.faults = faults or FakeFaults()
    self.rx: deque[CanFrame] = deque()
    self.closed = False
    self.isotp_expected = 0
    self.isotp_payload = bytearray()
    self.isotp_sequence = 1
    self.state = UpdateState.IDLE
    self.error = 0
    self.token = 0
    self.sequence = 0
    self.target_slot = SLOT_NONE
    self.offset = 0
    self.image_identity = b"\0\0\0"
    self.manifest: Manifest | None = None

  def _queue(self, address: int, data: bytes) -> None:
    self.rx.append(CanFrame(address, data.ljust(8, b"\0"), self.bus))

  def _queue_app(self, *, trial: bool = False) -> None:
    flags = (FLAG_TRIAL if trial else FLAG_CONFIRMED) | FLAG_UPDATE_SUPPORT
    self._queue(APP_INFO0_ID, bytes((1,)) + PRODUCT_ID.to_bytes(4, "little") +
                bytes((self.version.major, self.version.minor, self.version.patch)))
    self._queue(APP_INFO1_ID, bytes(((self.bootloader_version >> 16) & 0xFF,
                                    self.active_slot, flags, 1)) +
                self.build_identity.to_bytes(4, "little"))
    self._queue(APP_INFO2_ID, self.bootloader_version.to_bytes(4, "little") +
                self.security_version.to_bytes(4, "little"))
    self._queue(APP_INFO3_ID, bytes(8))

  def _single(self, payload: bytes, service: int) -> None:
    if self.faults.corrupt_next_response[service]:
      self.faults.corrupt_next_response[service] -= 1
      self._queue(BOOT_RESPONSE_ID, b"\x30\0\0")
    else:
      self._queue(BOOT_RESPONSE_ID, bytes((len(payload),)) + payload)

  def _status_base(self, service: int) -> bytes:
    return bytes((service | 0x40, int(self.state), self.error, self.confirmed_slot)) + self.offset.to_bytes(3, "little")

  def _handle(self, payload: bytes) -> None:
    service = payload[0]
    if self.faults.reject_next_request[service]:
      self.faults.reject_next_request[service] -= 1
      self._queue(BOOT_RESPONSE_ID, b"\x30\0\0")
      return
    if service == SVC_GET_INFO:
      page = payload[1] if len(payload) > 1 else 0
      if page == 0:
        response = bytes((service | 0x40, 1, 1, 1, self.confirmed_slot, SLOT_NONE, 0))
      elif page == 1:
        response = bytes((service | 0x40, 1)) + PRODUCT_ID.to_bytes(4, "little") + bytes((0x20,))
      else:
        response = bytes((service | 0x40, 2)) + self.bootloader_version.to_bytes(4, "little") + bytes((64,))
      self._single(response, service)
    elif service == SVC_STATUS:
      page = payload[1] if len(payload) > 1 else 0
      if page == 0:
        response = self._status_base(service)
      elif page == 1:
        response = bytes((service | 0x40, 1)) + self.token.to_bytes(4, "little") + bytes((self.sequence & 0xFF,))
      else:
        response = bytes((service | 0x40, 2, self.sequence >> 8, self.target_slot)) + self.image_identity
      self._single(response, service)
    elif service == SVC_START:
      token, sequence = struct.unpack_from("<IH", payload, 1)
      candidate = Manifest.parse(payload[7:])
      if (token == self.token and
          self.manifest is not None and candidate.raw == self.manifest.raw):
        self.error = 0
      elif self.state not in (UpdateState.IDLE, UpdateState.FAILED, UpdateState.COMMITTED):
        self.error = 13
      elif candidate.slot == self.confirmed_slot:
        self.error = 4
        self.state = UpdateState.FAILED
      else:
        self.token, self.sequence = token, sequence
        self.target_slot, self.offset = candidate.slot, 0
        self.image_identity = candidate.image_sha256[:3]
        self.manifest = candidate
        self.state, self.error = UpdateState.PROGRAMMING, 0
      self._single(self._status_base(service), service)
    elif service == SVC_DATA:
      token, sequence, offset, length = struct.unpack_from("<IHIH", payload, 1)
      data = payload[13:13 + length]
      crc = struct.unpack_from("<I", payload, 13 + length)[0]
      if token != self.token:
        self.error = 13
      elif sequence == self.sequence and offset + length == self.offset:
        pass
      elif sequence != ((self.sequence + 1) & 0xFFFF) or offset != self.offset:
        self.error = 14
      elif binascii.crc32(data) & 0xFFFFFFFF != crc:
        self.error = 23
      else:
        self.offset += length
        self.sequence = sequence
        if self.manifest is not None and self.offset == self.manifest.image_size:
          self.state = UpdateState.PROGRAMMED
      self._single(self._status_base(service), service)
    elif service == SVC_FINISH:
      token, sequence = struct.unpack_from("<IH", payload, 1)
      if self.state == UpdateState.COMMITTED and token == self.token and sequence == self.sequence:
        pass
      elif token != self.token or sequence != ((self.sequence + 1) & 0xFFFF):
        self.error = 14
      elif self.faults.verification_error:
        self.error = self.faults.verification_error
        self.state = UpdateState.FAILED
      else:
        self.sequence = sequence
        self.state = UpdateState.COMMITTED
      self._single(self._status_base(service), service)
    elif service == SVC_ABORT:
      if int.from_bytes(payload[1:5], "little") == self.token:
        self.state, self.error = UpdateState.FAILED, 22
      else:
        self.error = 13
      self._single(self._status_base(service), service)
    elif service == SVC_RESET:
      self._single(bytes((service | 0x40,)), service)
      if self.state == UpdateState.COMMITTED and self.manifest is not None and not self.faults.rollback:
        self.active_slot = self.target_slot
        self.version = self.manifest.version
        self.build_identity = self.manifest.build_identity
        self.security_version = self.manifest.security_version
        self.mode = "app"
        self._queue_app(trial=True)
        self._queue_app(trial=False)
      elif self.confirmed_slot != SLOT_NONE:
        self.active_slot = self.confirmed_slot
        self.mode = "app"
        self._queue_app()

  def send(self, address: int, data: bytes, bus: int) -> None:
    if self.closed or bus != self.bus or len(data) != 8:
      raise ValueError("invalid fake CAN send")
    if address == ENTER_BOOT_ID and self.mode == "app":
      self.mode = "boot"
      self.rx.clear()
      return
    if address != BOOT_REQUEST_ID or self.mode != "boot":
      return
    pci = data[0] >> 4
    if pci == 0:
      self._handle(data[1:1 + (data[0] & 0xF)])
    elif pci == 1:
      self.isotp_expected = ((data[0] & 0xF) << 8) | data[1]
      self.isotp_payload = bytearray(data[2:8])
      self.isotp_sequence = 1
      self._queue(BOOT_RESPONSE_ID, b"\x30\0\0")
    elif pci == 2 and (data[0] & 0xF) == self.isotp_sequence:
      needed = self.isotp_expected - len(self.isotp_payload)
      self.isotp_payload.extend(data[1:1 + min(7, needed)])
      self.isotp_sequence = (self.isotp_sequence + 1) & 0xF
      if len(self.isotp_payload) == self.isotp_expected:
        payload = bytes(self.isotp_payload)
        self.isotp_expected = 0
        self._handle(payload)

  def recv(self, timeout: float) -> CanFrame | None:
    del timeout
    if not self.rx and self.mode == "app":
      self._queue_app()
    return self.rx.popleft() if self.rx else None

  def flush(self) -> None:
    self.rx.clear()

  def close(self) -> None:
    self.closed = True
    self.rx.clear()
