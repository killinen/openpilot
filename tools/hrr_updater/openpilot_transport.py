from __future__ import annotations

import time
from collections import deque

from typing import Any

from openpilot.tools.hrr_updater.errors import TransportError
from openpilot.tools.hrr_updater.transport import CanFrame, CanTransport


class OpenpilotPandaTransport(CanTransport):
  """CAN through cereal while pandad remains the sole Panda USB owner."""

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
    return [CanFrame(msg.address, bytes(msg.dat), msg.src, msg.busTime) for msg in event.can]

  def send(self, address: int, data: bytes, bus: int) -> None:
    if self.closed:
      raise TransportError("CAN transport is closed")
    if len(data) > 8:
      raise TransportError("HRR transport supports classic CAN only")
    if self.sendcan is None:
      try:
        self.sendcan = self.messaging.PubMaster(["sendcan"])
      except Exception as exc:
        raise TransportError("cannot acquire cereal sendcan; another publisher may be active") from exc
    message = [(address, 0, data, bus)]
    self.sendcan.send("sendcan", self.can_list_to_can_capnp(message, msgtype="sendcan"))
    deadline = time.monotonic() + self.receipt_timeout
    while time.monotonic() < deadline:
      for frame in self._receive_event(deadline - time.monotonic()):
        if frame.address == address and frame.data == data and frame.src == (0xC0 | bus):
          raise TransportError(f"Panda safety rejected 0x{address:03x} on bus {bus}")
        if frame.address == address and frame.data == data and frame.src == (0x80 | bus):
          return
        self.pending.append(frame)
    raise TransportError(f"no Panda TX receipt for 0x{address:03x} on bus {bus}")

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
