#!/usr/bin/env python3
"""Guided live calibration for the STM32G474 HRR resolver-angle estimator."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import select
import struct
import sys
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
  from panda import Panda


CONFIG_ADDR = 0x603
ANGLE_STATUS_ADDR = 0x632
CAL_STATUS_ADDR = 0x635
CONFIG_KEY = 0xA5

CMD_CAL_START = 4
CMD_CAL_FINISH_SAVE = 5
CMD_CAL_ABORT = 6
CMD_CAL_MODE = 7

MIN_SAMPLES = 100
MIN_SPAN_DEG = 150.0
STATUS_TIMEOUT_S = 1.0


def crc8_poly07(data: bytes) -> int:
  crc = 0
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


def build_config_frame(command: int, value: int = 0) -> bytes:
  if not 0 <= command <= 0xFF:
    raise ValueError("command must fit in one byte")
  if not 0 <= value <= 0xFFFFFFFF:
    raise ValueError("value must fit in an unsigned 32-bit field")
  body = bytes((command,)) + value.to_bytes(4, "little") + bytes((CONFIG_KEY, 0))
  checksum = crc8_poly07(bytes((CONFIG_ADDR & 0xFF, CONFIG_ADDR >> 8)) + body)
  return body + bytes((checksum,))


@dataclass(frozen=True)
class CalibrationStatus:
  running: bool
  valid: bool
  enabled: bool
  failed: bool
  in_raw_valid: bool
  ou_raw_valid: bool
  legacy_active: bool
  version: int
  samples: int
  in_span_deg: float
  ou_span_deg: float

  @classmethod
  def decode(cls, payload: bytes) -> CalibrationStatus:
    if len(payload) != 8:
      raise ValueError(f"expected 8-byte 0x{CAL_STATUS_ADDR:03X}, got {len(payload)}")
    flags, version, samples, in_span, ou_span = struct.unpack("<BBHHH", payload)
    return cls(
      running=bool(flags & (1 << 0)),
      valid=bool(flags & (1 << 1)),
      enabled=bool(flags & (1 << 2)),
      failed=bool(flags & (1 << 3)),
      in_raw_valid=bool(flags & (1 << 4)),
      ou_raw_valid=bool(flags & (1 << 5)),
      legacy_active=bool(flags & (1 << 6)),
      version=version,
      samples=samples,
      in_span_deg=in_span * 0.1,
      ou_span_deg=ou_span * 0.1,
    )

  @property
  def ready(self) -> bool:
    return self.samples >= MIN_SAMPLES and self.in_span_deg >= MIN_SPAN_DEG and self.ou_span_deg >= MIN_SPAN_DEG

  def format(self) -> str:
    mode = "LEGACY" if self.legacy_active else "CALIBRATED"
    return (
      f"samples={self.samples}/{MIN_SAMPLES} "
      f"IN_span={self.in_span_deg:5.1f}/{MIN_SPAN_DEG:.1f}deg "
      f"OU_span={self.ou_span_deg:5.1f}/{MIN_SPAN_DEG:.1f}deg "
      f"raw_valid={int(self.in_raw_valid)}/{int(self.ou_raw_valid)} "
      f"state={'RUNNING' if self.running else 'IDLE'} mode={mode}"
    )


@dataclass(frozen=True)
class AngleStatus:
  ou_angle_deg: float
  in_angle_deg: float

  @classmethod
  def decode(cls, payload: bytes) -> AngleStatus:
    if len(payload) != 8:
      raise ValueError(f"expected 8-byte 0x{ANGLE_STATUS_ADDR:03X}, got {len(payload)}")
    _, _, ou_angle, in_angle = struct.unpack("<hhHH", payload)
    return cls(ou_angle * 0.1, in_angle * 0.1)


class HrrCalibrationSession:
  def __init__(self, panda: Panda | None, bus: int, dry_run: bool) -> None:
    self.panda = panda
    self.bus = bus
    self.dry_run = dry_run
    self.calibration: CalibrationStatus | None = None
    self.angles: AngleStatus | None = None
    self.last_status_at: float | None = None

  def send_command(self, command: int, value: int = 0) -> bytes:
    payload = build_config_frame(command, value)
    print(f"TX 0x{CONFIG_ADDR:03X}: {payload.hex(' ')}")
    if not self.dry_run:
      assert self.panda is not None
      self.panda.can_send(CONFIG_ADDR, payload, self.bus)
    return payload

  def poll(self) -> None:
    if self.dry_run:
      return
    assert self.panda is not None
    for address, _, payload, rx_bus in self.panda.can_recv():
      if rx_bus != self.bus:
        continue
      try:
        if address == CAL_STATUS_ADDR:
          self.calibration = CalibrationStatus.decode(payload)
          self.last_status_at = time.monotonic()
        elif address == ANGLE_STATUS_ADDR:
          self.angles = AngleStatus.decode(payload)
      except ValueError:
        continue

  def wait_for_status(self, timeout: float) -> CalibrationStatus | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      self.poll()
      if self.calibration is not None and self.last_status_at is not None:
        if time.monotonic() - self.last_status_at <= STATUS_TIMEOUT_S:
          return self.calibration
      time.sleep(0.01)
    return None

  def wait_for(self, predicate, timeout: float) -> CalibrationStatus | None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      self.poll()
      status = self.calibration
      if status is not None and predicate(status):
        return status
      time.sleep(0.02)
    return None


def choose_bus(configured_bus: int | None) -> int:
  if configured_bus is not None:
    return configured_bus
  while True:
    value = input("Panda CAN bus [0/1/2]: ").strip()
    if value in {"0", "1", "2"}:
      return int(value)
    print("Enter 0, 1, or 2.")


def run_guided(session: HrrCalibrationSession, timeout: float, assume_yes: bool) -> bool:
  print("\nResolver calibration must be performed with HRR torque output disengaged.")
  print("Secure the vehicle, keep clear of the steering mechanism, and move the wheel manually and slowly.")
  print("The sweep must cover at least one complete resolver phase in both IN and OU.")
  if not assume_yes:
    input("Press Enter when ready, or Ctrl-C to cancel: ")

  session.send_command(CMD_CAL_START)
  if session.dry_run:
    print("Dry run: start frame generated; no live calibration performed.")
    return True

  started = session.wait_for(lambda status: status.running, 2.0)
  if started is None:
    print("ERROR: HRR did not acknowledge calibration start on 0x635.", file=sys.stderr)
    session.send_command(CMD_CAL_ABORT)
    return False

  print("\nSlowly sweep full left -> full right -> full left. Press Enter once READY is shown.")
  deadline = time.monotonic() + timeout
  next_print = 0.0
  user_finished = False
  while time.monotonic() < deadline:
    session.poll()
    status = session.calibration
    now = time.monotonic()
    if status is not None and now >= next_print:
      angles = ""
      if session.angles is not None:
        angles = f" IN={session.angles.in_angle_deg:5.1f}deg OU={session.angles.ou_angle_deg:5.1f}deg"
      print(f"\r{'READY' if status.ready else 'SWEEP'} {status.format()}{angles}    ", end="", flush=True)
      next_print = now + 0.2
    if sys.stdin.isatty():
      readable, _, _ = select.select([sys.stdin], [], [], 0)
      if readable:
        sys.stdin.readline()
        user_finished = True
    elif status is not None and status.ready:
      user_finished = True
    if user_finished:
      print()
      if status is not None and status.ready:
        break
      print("Coverage is not sufficient yet; continue sweeping and press Enter when READY is shown.")
      user_finished = False
    time.sleep(0.02)
  else:
    print("\nERROR: calibration timed out; aborting without replacing stored limits.", file=sys.stderr)
    session.send_command(CMD_CAL_ABORT)
    return False

  session.send_command(CMD_CAL_FINISH_SAVE)
  finished = session.wait_for(lambda item: not item.running, 3.0)
  if finished is None:
    print("ERROR: HRR did not report calibration completion.", file=sys.stderr)
    return False
  print(f"Result: {finished.format()}")
  if finished.failed or not finished.valid or not finished.enabled or finished.legacy_active:
    print("Calibration failed; firmware is using legacy uncalibrated angles.", file=sys.stderr)
    return False
  print("Calibration saved and calibrated resolver-angle output enabled.")
  return True


def run_self_test() -> None:
  for command, value in ((CMD_CAL_START, 0), (CMD_CAL_FINISH_SAVE, 0),
                         (CMD_CAL_ABORT, 0), (CMD_CAL_MODE, 1)):
    frame = build_config_frame(command, value)
    assert len(frame) == 8
    assert crc8_poly07(bytes((CONFIG_ADDR & 0xFF, CONFIG_ADDR >> 8)) + frame[:7]) == frame[7]

  status = CalibrationStatus.decode(struct.pack("<BBHHH", 0x37, 1, 123, 1600, 1550))
  assert status.running and status.valid and status.enabled
  assert status.in_raw_valid and status.ou_raw_valid and not status.legacy_active
  assert status.ready
  angles = AngleStatus.decode(struct.pack("<hhHH", 0, 0, 1234, 567))
  assert angles.ou_angle_deg == 123.4 and angles.in_angle_deg == 56.7
  print("HRR resolver-calibration protocol self-test passed.")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), help="Panda CAN bus; prompted when omitted")
  parser.add_argument("--timeout", type=float, default=300.0, help="maximum guided sweep duration")
  parser.add_argument("--yes", action="store_true", help="skip the initial safety confirmation")
  parser.add_argument("--legacy", action="store_true", help="select persisted legacy uncalibrated output and exit")
  parser.add_argument("--calibrated", action="store_true", help="select stored calibrated output and exit")
  parser.add_argument("--abort", action="store_true", help="abort an active calibration and exit")
  parser.add_argument("--dry-run", action="store_true", help="print command frames without opening Panda")
  parser.add_argument("--self-test", action="store_true", help="verify frame and status encodings")
  args = parser.parse_args()
  if args.self_test:
    run_self_test()
    return
  if args.timeout <= 0:
    parser.error("--timeout must be greater than zero")
  if sum((args.legacy, args.calibrated, args.abort)) > 1:
    parser.error("--legacy, --calibrated, and --abort are mutually exclusive")

  bus = choose_bus(args.bus)
  panda = None
  if not args.dry_run:
    from panda import Panda
    panda = Panda()
    panda.set_power_save(False)
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  session = HrrCalibrationSession(panda, bus, args.dry_run)
  try:
    if not args.dry_run and session.wait_for_status(3.0) is None:
      print(f"ERROR: no HRR calibration status 0x{CAL_STATUS_ADDR:03X} on bus {bus}.", file=sys.stderr)
      raise SystemExit(2)
    if args.legacy:
      session.send_command(CMD_CAL_MODE, 0)
    elif args.calibrated:
      session.send_command(CMD_CAL_MODE, 1)
    elif args.abort:
      session.send_command(CMD_CAL_ABORT)
    elif not run_guided(session, args.timeout, args.yes):
      raise SystemExit(1)
  except (KeyboardInterrupt, EOFError):
    print("\nCalibration aborted; existing stored calibration remains unchanged.")
    session.send_command(CMD_CAL_ABORT)
  finally:
    if panda is not None:
      panda.set_safety_mode(Panda.SAFETY_SILENT)


if __name__ == "__main__":
  main()
