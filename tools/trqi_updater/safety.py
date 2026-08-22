from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol

from openpilot.tools.trqi_updater.errors import UnsafeStateError


def ignition_interlock_active(startup_hold: bool, in_progress: bool) -> bool:
  """True while either the preflight hold or durable recovery blocks ONROAD."""
  return startup_hold or in_progress


class SafetyGate(Protocol):
  def check(self) -> None: ...
  def acquire(self, timeout: float = 10.0) -> None: ...
  def mark_in_progress(self, *, recovery_safe: bool = False) -> None: ...
  def set_recovery_safe(self, recovery_safe: bool) -> None: ...
  def check_during_update(self) -> None: ...
  def release(self, *, preserve_in_progress: bool = False) -> None: ...


@dataclass
class FakeSafetyGate:
  safe: bool = True
  ignition: bool = False
  allow_ignition: bool = False
  acquired: bool = False
  in_progress: bool = False
  recovery_safe: bool = False

  def check(self) -> None:
    if not self.safe or (self.ignition and not self.allow_ignition):
      raise UnsafeStateError("fake host state is unsafe")

  def acquire(self, timeout: float = 10.0) -> None:
    del timeout
    self.check()
    self.acquired = True

  def mark_in_progress(self, *, recovery_safe: bool = False) -> None:
    self.check()
    if not self.acquired:
      raise UnsafeStateError("update lease was not acquired")
    self.in_progress = True
    self.recovery_safe = recovery_safe

  def set_recovery_safe(self, recovery_safe: bool) -> None:
    self.recovery_safe = recovery_safe

  def check_during_update(self) -> None:
    self.check()

  def release(self, *, preserve_in_progress: bool = False) -> None:
    self.acquired = False
    if not preserve_in_progress:
      self.in_progress = False
      self.recovery_safe = False


class OpenpilotSafetyGate:
  SAFETY_TRQI_UPDATER = 32
  UPDATE_DEADLINE_SECONDS = 600

  def __init__(self, bus: int):
    try:
      import cereal.messaging as messaging
      from openpilot.common.params import Params
    except Exception as exc:
      raise UnsafeStateError("openpilot state APIs are unavailable") from exc
    self.messaging = messaging
    self.params = Params()
    self.bus = bus
    self.sm = messaging.SubMaster(["deviceState", "pandaStates", "controlsState", "carState"],
                                  ignore_alive=["controlsState", "carState"])
    self.acquired = False
    self.in_progress = self.params.get_bool("TrqiUpdateInProgress")

  def _update(self, timeout_ms: int = 1000) -> None:
    self.sm.update(timeout_ms)

  def _panda(self):
    panda_index = self.bus // 4
    states = self.sm["pandaStates"]
    if panda_index >= len(states):
      raise UnsafeStateError(f"Panda for global bus {self.bus} is unavailable")
    return states[panda_index]

  def check(self) -> None:
    self._update()
    if not self.sm.seen["deviceState"] or not self.sm.alive["deviceState"]:
      raise UnsafeStateError("deviceState is not fresh")
    if self.sm["deviceState"].started or self.params.get_bool("IsOnroad") or not self.params.get_bool("IsOffroad"):
      raise UnsafeStateError("openpilot is ONROAD")
    if self.params.get_bool("IsEngaged"):
      raise UnsafeStateError("controls are engaged")
    if self.sm.seen["controlsState"] and self.sm.alive["controlsState"] and self.sm["controlsState"].enabled:
      raise UnsafeStateError("controlsState is enabled")
    if self.sm.seen["carState"] and self.sm.alive["carState"] and abs(self.sm["carState"].vEgo) > 0.05:
      raise UnsafeStateError("vehicle speed is nonzero")
    panda = self._panda()
    ignition = panda.ignitionLine or panda.ignitionCan
    if ignition and not ignition_interlock_active(self.params.get_bool("TrqiUpdateStartupHold"),
                                                  self.params.get_bool("TrqiUpdateInProgress")):
      raise UnsafeStateError("ignition is active without a TRQI pre-ONROAD startup hold")
    if panda.controlsAllowed:
      raise UnsafeStateError("Panda controls are allowed")

  def acquire(self, timeout: float = 10.0) -> None:
    self.check()
    self.params.put_bool("TrqiUpdaterRequested", True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      self._update(250)
      panda = self._panda()
      if (self.params.get_bool("TrqiUpdaterReady") and panda.safetyModel.raw == self.SAFETY_TRQI_UPDATER and
          not panda.powerSaveEnabled):
        self.acquired = True
        return
    self.params.put_bool("TrqiUpdaterRequested", False)
    raise UnsafeStateError("pandad did not grant the TRQI pre-ONROAD update lease")

  def mark_in_progress(self, *, recovery_safe: bool = False) -> None:
    self.check()
    if not self.acquired:
      raise UnsafeStateError("TRQI update lease is not active")
    self.params.put("TrqiUpdateDeadline", str(int(time.time()) + self.UPDATE_DEADLINE_SECONDS))
    self.params.put_bool("TrqiUpdateInProgress", True)
    self.params.put_bool("TrqiUpdateRecoverySafe", recovery_safe)
    self.in_progress = True

  def set_recovery_safe(self, recovery_safe: bool) -> None:
    self.params.put_bool("TrqiUpdateRecoverySafe", recovery_safe)

  def check_during_update(self) -> None:
    self._update(0)
    if self.sm["deviceState"].started or self.params.get_bool("IsOnroad") or self.params.get_bool("IsEngaged"):
      raise UnsafeStateError("openpilot entered an unsafe state during the TRQI update")
    panda = self._panda()
    if ((panda.ignitionLine or panda.ignitionCan) and not ignition_interlock_active(
        self.params.get_bool("TrqiUpdateStartupHold"), self.params.get_bool("TrqiUpdateInProgress"))):
      raise UnsafeStateError("TRQI OFFROAD interlock was released during the update")
    if panda.safetyModel.raw != self.SAFETY_TRQI_UPDATER or panda.powerSaveEnabled:
      raise UnsafeStateError("Panda left TRQI updater safety mode")
    # hardwared uses this renewable deadline only as a final crash guard. The
    # bootloader independently resets to the confirmed slot after host silence.
    self.params.put("TrqiUpdateDeadline", str(int(time.time()) + self.UPDATE_DEADLINE_SECONDS))

  def release(self, *, preserve_in_progress: bool = False) -> None:
    if not preserve_in_progress:
      self.params.put_bool("TrqiUpdateInProgress", False)
      self.params.remove("TrqiUpdateDeadline")
      self.params.put_bool("TrqiUpdateStartupHold", False)
      self.params.put_bool("TrqiUpdateRecoverySafe", False)
    self.params.put_bool("TrqiUpdaterRequested", False)
    self.params.put_bool("TrqiUpdaterReady", False)
    self.in_progress = preserve_in_progress
    self.acquired = False
