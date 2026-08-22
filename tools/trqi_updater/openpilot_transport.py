from __future__ import annotations

import time
from collections import deque

from typing import Any

from openpilot.tools.trqi_updater.errors import TransportError, TransportTimeout
from openpilot.tools.trqi_updater.transport import CanFrame, CanTransport


class OpenpilotPandaTransport(CanTransport):
  """CAN through cereal while pandad remains the sole Panda USB owner."""

  # Keep unrelated vehicle traffic out of the pending queue. TX receipts use
  # the request IDs, while the remaining IDs are the only TRQI responses and
  # application identity frames consumed by this package.
  RELEVANT_ADDRESSES = {0x60A, 0x639, 0x63A, 0x63B, 0x63C, 0x6A0, 0x6A1}
  SENDCAN_READER_TIMEOUT = 2

  def __init__(self, *, receipt_timeout: float = 0.75):
    try:
      import cereal.messaging as messaging
      from openpilot.selfdrive.pandad import can_list_to_can_capnp
      self.messaging = messaging
      self.can_list_to_can_capnp = can_list_to_can_capnp
      self.can_sock = messaging.sub_sock("can", conflate=False)
    except Exception as exc:
      raise TransportError("cannot subscribe to cereal can") from exc
    self.sendcan: Any | None = None
    self.receipt_timeout = receipt_timeout
    self.pending: deque[CanFrame] = deque()
    self.closed = False

  def _receive_event(self, timeout: float) -> list[CanFrame]:
    if self.closed:
      return []
    self.can_sock.setTimeout(max(0, int(timeout * 1000)))
    raw = self.can_sock.receive()
    if raw is None:
      return []
    event = self.messaging.log_from_bytes(raw)
    return [CanFrame(msg.address, bytes(msg.dat), msg.src, msg.busTime) for msg in event.can
            if msg.address in self.RELEVANT_ADDRESSES]

  def send(self, address: int, data: bytes, bus: int) -> None:
    if self.closed:
      raise TransportError("CAN transport is closed")
    if len(data) > 8:
      raise TransportError("TRQI transport supports classic CAN only")
    if self.sendcan is None:
      try:
        self.sendcan = self.messaging.PubMaster(["sendcan"])
      except Exception as exc:
        raise TransportError("cannot acquire cereal sendcan; another publisher may be active") from exc
      # Creating an MSGQ publisher resets its reader registry. Pandad's
      # sendcan subscriber must reattach before the first critical frame.
      try:
        reader_ready = self.sendcan.wait_for_readers_to_update(
          "sendcan", self.SENDCAN_READER_TIMEOUT)
      except Exception as exc:
        self.sendcan = None
        raise TransportError("cannot verify pandad cereal sendcan readiness") from exc
      if not reader_ready:
        self.sendcan = None
        raise TransportError("pandad did not connect to cereal sendcan")
    message = [(address, 0, data, bus)]
    self.sendcan.send("sendcan", self.can_list_to_can_capnp(message, msgtype="sendcan"))
    deadline = time.monotonic() + self.receipt_timeout
    while time.monotonic() < deadline:
      tx_accepted = False
      tx_rejected = False
      for frame in self._receive_event(deadline - time.monotonic()):
        if frame.address == address and frame.data == data and frame.src == (0xC0 | bus):
          tx_rejected = True
        elif frame.address == address and frame.data == data and frame.src == (0x80 | bus):
          tx_accepted = True
        else:
          self.pending.append(frame)
      if tx_rejected:
        raise TransportError(f"Panda safety rejected 0x{address:03x} on bus {bus}")
      if tx_accepted:
        return
    raise TransportTimeout(f"no Panda TX receipt for 0x{address:03x} on bus {bus}")

  def recv(self, timeout: float) -> CanFrame | None:
    if self.pending:
      return self.pending.popleft()
    frames = self._receive_event(timeout)
    self.pending.extend(frames)
    return self.pending.popleft() if self.pending else None

  def flush(self) -> None:
    self.pending.clear()
    if self.closed:
      return
    while self._receive_event(0):
      pass

  def close(self) -> None:
    self.closed = True
    self.pending.clear()
    self.sendcan = None
    self.can_sock = None
