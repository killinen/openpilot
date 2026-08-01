from __future__ import annotations

import binascii
import secrets
import struct
import time
from dataclasses import dataclass
from enum import IntEnum

from openpilot.tools.hrr_updater.constants import (APP_INFO0_ID, APP_INFO1_ID, APP_INFO2_ID, APP_INFO3_ID,
                                         BOOT_REQUEST_ID, BOOT_RESPONSE_ID, ENTER_BOOT_ID, FLAG_CONFIRMED,
                                         FLAG_TRIAL, PROTOCOL_VERSION, SLOT_NONE, SVC_ABORT, SVC_DATA,
                                         SVC_FINISH, SVC_GET_INFO, SVC_RESET, SVC_START, SVC_STATUS)
from openpilot.tools.hrr_updater.errors import BootloaderRefused, TransportError
from openpilot.tools.hrr_updater.manifest import Manifest, Version
from openpilot.tools.hrr_updater.transport import CanFrame, CanTransport


class UpdateState(IntEnum):
  IDLE = 0
  RECEIVED = 1
  VALIDATED = 2
  ERASING = 3
  PROGRAMMING = 4
  PROGRAMMED = 5
  VERIFIED = 6
  COMMITTED = 7
  FAILED = 8


BOOT_ERRORS = {
  0: "OK", 1: "INVALID_METADATA", 2: "INVALID_MANIFEST", 3: "WRONG_PRODUCT",
  4: "WRONG_SLOT", 5: "IMAGE_TOO_LARGE", 6: "BAD_VECTOR", 7: "ERASE",
  8: "PROGRAM", 9: "VERIFY", 10: "SHA", 11: "SIGNATURE", 12: "ROLLBACK",
  13: "SESSION", 14: "SEQUENCE", 15: "CAN_TIMEOUT", 16: "FLASH_RESET",
  17: "CANDIDATE", 18: "NO_IMAGE", 19: "GEOMETRY", 20: "OFFSET",
  21: "DIFFERENT_DUPLICATE", 22: "STATE", 23: "CRC",
}


@dataclass(frozen=True)
class AppInfo:
  protocol_version: int
  product_id: int
  version: Version
  bootloader_version: int
  active_slot: int
  flags: int
  update_supported: bool
  build_identity: int
  security_version: int | None = None
  last_boot_error: int | None = None
  reset_flags: int | None = None

  @property
  def confirmed(self) -> bool:
    return bool(self.flags & FLAG_CONFIRMED)

  @property
  def trial(self) -> bool:
    return bool(self.flags & FLAG_TRIAL)


@dataclass(frozen=True)
class BootInfo:
  protocol_version: int
  bootloader_version: int
  product_id: int
  confirmed_slot: int
  pending_slot: int
  geometry_error: int
  flash_size: int
  max_chunk: int


@dataclass(frozen=True)
class BootStatus:
  state: UpdateState
  error: int
  confirmed_slot: int
  next_offset: int
  session_token: int = 0
  sequence: int = 0
  target_slot: int = SLOT_NONE
  image_identity_prefix: bytes = b""

  @property
  def error_name(self) -> str:
    return BOOT_ERRORS.get(self.error, f"UNKNOWN_{self.error}")


def crc8_poly07(data: bytes) -> int:
  crc = 0
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


def build_enter_bootloader(nonce: int) -> bytes:
  if not 0 < nonce <= 0xFFFFFFFF:
    raise ValueError("entry nonce must be a nonzero uint32")
  prefix = bytes((0xB0, PROTOCOL_VERSION)) + nonce.to_bytes(4, "little") + bytes((0xA5,))
  return prefix + bytes((crc8_poly07(prefix),))


class IsoTpClient:
  """Small request-side ISO-TP implementation with deliberate busy-bus pacing."""

  def __init__(self, transport: CanTransport, bus: int, *, frame_pacing: float = 0.003):
    self.transport = transport
    self.bus = bus
    self.frame_pacing = max(0.0, frame_pacing)

  def _recv_boot_frame(self, timeout: float) -> CanFrame:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      frame = self.transport.recv(max(0.0, deadline - time.monotonic()))
      if frame is not None and frame.src == self.bus and frame.address == BOOT_RESPONSE_ID:
        return frame
    raise TransportError("timeout waiting for HRR bootloader response")

  def request(self, payload: bytes, *, timeout: float = 2.0) -> bytes:
    if not 1 <= len(payload) <= 600:
      raise ValueError("ISO-TP payload is outside bootloader limits")
    if len(payload) <= 7:
      self.transport.send(BOOT_REQUEST_ID, (bytes((len(payload),)) + payload).ljust(8, b"\0"), self.bus)
    else:
      length = len(payload)
      first = bytes((0x10 | ((length >> 8) & 0x0F), length & 0xFF)) + payload[:6]
      self.transport.send(BOOT_REQUEST_ID, first, self.bus)
      flow = self._recv_boot_frame(timeout)
      if len(flow.data) < 3 or flow.data[0] != 0x30:
        raise TransportError(f"invalid ISO-TP flow control: {flow.data.hex()}")
      block_size = flow.data[1]
      stmin_raw = flow.data[2]
      if stmin_raw <= 0x7F:
        receiver_pacing = stmin_raw / 1000.0
      elif 0xF1 <= stmin_raw <= 0xF9:
        receiver_pacing = (stmin_raw - 0xF0) / 10000.0
      else:
        raise TransportError(f"unsupported ISO-TP STmin 0x{stmin_raw:02x}")
      pacing = max(self.frame_pacing, receiver_pacing)
      sequence = 1
      sent_in_block = 0
      for offset in range(6, length, 7):
        frame = (bytes((0x20 | (sequence & 0x0F),)) + payload[offset:offset + 7]).ljust(8, b"\0")
        if pacing:
          time.sleep(pacing)
        self.transport.send(BOOT_REQUEST_ID, frame, self.bus)
        sequence = (sequence + 1) & 0x0F
        sent_in_block += 1
        if block_size and sent_in_block == block_size and offset + 7 < length:
          flow = self._recv_boot_frame(timeout)
          if len(flow.data) < 3 or flow.data[0] != 0x30:
            raise TransportError("missing ISO-TP continuation flow control")
          sent_in_block = 0

    response = self._recv_boot_frame(timeout)
    if not response.data or response.data[0] >> 4 != 0:
      raise TransportError(f"bootloader response is not ISO-TP single-frame: {response.data.hex()}")
    response_length = response.data[0] & 0x0F
    if not 1 <= response_length <= 7 or len(response.data) < response_length + 1:
      raise TransportError("malformed bootloader response")
    return response.data[1:1 + response_length]


class HrrProtocol:
  def __init__(self, transport: CanTransport, bus: int, *, frame_pacing: float = 0.003):
    self.transport = transport
    self.bus = bus
    self.isotp = IsoTpClient(transport, bus, frame_pacing=frame_pacing)

  def enter_bootloader(self) -> int:
    nonce = secrets.randbits(32) or 1
    self.transport.send(ENTER_BOOT_ID, build_enter_bootloader(nonce), self.bus)
    return nonce

  def _expect(self, service: int, response: bytes) -> bytes:
    if not response or response[0] != (service | 0x40):
      raise BootloaderRefused(f"unexpected response to 0x{service:02x}: {response.hex()}")
    return response

  def get_info(self) -> BootInfo:
    pages = [self._expect(SVC_GET_INFO, self.isotp.request(bytes((SVC_GET_INFO, page)))) for page in range(3)]
    if len(pages[0]) != 7 or len(pages[1]) != 7 or len(pages[2]) != 7:
      raise TransportError("incomplete bootloader information")
    protocol_version, bl_major, bl_minor, confirmed, pending, geometry = pages[0][1:]
    product_id = int.from_bytes(pages[1][2:6], "little")
    flash_size = pages[1][6] * 16 * 1024
    bootloader_version = int.from_bytes(pages[2][2:6], "little")
    max_chunk = pages[2][6] * 8
    if bootloader_version == 0:
      bootloader_version = (bl_major << 16) | (bl_minor << 8)
    return BootInfo(protocol_version, bootloader_version, product_id, confirmed, pending, geometry, flash_size, max_chunk)

  @staticmethod
  def _parse_status(response: bytes) -> BootStatus:
    if len(response) != 7:
      raise TransportError(f"malformed status response: {response.hex()}")
    return BootStatus(UpdateState(response[1]), response[2], response[3], int.from_bytes(response[4:7], "little"))

  def status(self) -> BootStatus:
    page0 = self._expect(SVC_STATUS, self.isotp.request(bytes((SVC_STATUS, 0))))
    base = self._parse_status(page0)
    page1 = self._expect(SVC_STATUS, self.isotp.request(bytes((SVC_STATUS, 1))))
    page2 = self._expect(SVC_STATUS, self.isotp.request(bytes((SVC_STATUS, 2))))
    if len(page1) != 7 or len(page2) != 7:
      raise TransportError("malformed extended status")
    token = int.from_bytes(page1[2:6], "little")
    sequence = page1[6] | (page2[2] << 8)
    return BootStatus(base.state, base.error, base.confirmed_slot, base.next_offset,
                      token, sequence, page2[3], bytes(page2[4:7]))

  def start(self, token: int, sequence: int, manifest: Manifest, *, timeout: float = 20.0) -> BootStatus:
    payload = bytes((SVC_START,)) + struct.pack("<IH", token, sequence) + manifest.raw
    return self._parse_status(self._expect(SVC_START, self.isotp.request(payload, timeout=timeout)))

  def data(self, token: int, sequence: int, offset: int, data: bytes, *, timeout: float = 3.0) -> BootStatus:
    payload = (bytes((SVC_DATA,)) + struct.pack("<IHIH", token, sequence, offset, len(data)) + data +
               struct.pack("<I", binascii.crc32(data) & 0xFFFFFFFF))
    return self._parse_status(self._expect(SVC_DATA, self.isotp.request(payload, timeout=timeout)))

  def finish(self, token: int, sequence: int, *, timeout: float = 15.0) -> BootStatus:
    payload = bytes((SVC_FINISH,)) + struct.pack("<IH", token, sequence)
    return self._parse_status(self._expect(SVC_FINISH, self.isotp.request(payload, timeout=timeout)))

  def abort(self, token: int) -> BootStatus:
    payload = bytes((SVC_ABORT,)) + struct.pack("<I", token)
    return self._parse_status(self._expect(SVC_ABORT, self.isotp.request(payload)))

  def reset(self) -> None:
    self._expect(SVC_RESET, self.isotp.request(bytes((SVC_RESET,))))


class AppInfoCollector:
  def __init__(self, bus: int):
    self.bus = bus
    self.frames: dict[int, bytes] = {}

  def add(self, frame: CanFrame) -> AppInfo | None:
    if frame.src != self.bus or frame.address not in (APP_INFO0_ID, APP_INFO1_ID, APP_INFO2_ID, APP_INFO3_ID):
      return None
    if len(frame.data) != 8:
      return None
    self.frames[frame.address] = frame.data
    return self.snapshot()

  @property
  def complete(self) -> bool:
    return all(address in self.frames for address in
               (APP_INFO0_ID, APP_INFO1_ID, APP_INFO2_ID, APP_INFO3_ID))

  def snapshot(self) -> AppInfo | None:
    if APP_INFO0_ID not in self.frames or APP_INFO1_ID not in self.frames:
      return None
    info0, info1 = self.frames[APP_INFO0_ID], self.frames[APP_INFO1_ID]
    info2 = self.frames.get(APP_INFO2_ID)
    info3 = self.frames.get(APP_INFO3_ID)
    bootloader_version = int.from_bytes(info2[:4], "little") if info2 else info1[0] << 16
    security_version = int.from_bytes(info2[4:8], "little") if info2 else None
    last_error = int.from_bytes(info3[:4], "little") if info3 else None
    reset_flags = int.from_bytes(info3[4:8], "little") if info3 else None
    return AppInfo(info0[0], int.from_bytes(info0[1:5], "little"), Version(*info0[5:8]),
                   bootloader_version, info1[1], info1[2], bool(info1[3]),
                   int.from_bytes(info1[4:8], "little"), security_version, last_error, reset_flags)
