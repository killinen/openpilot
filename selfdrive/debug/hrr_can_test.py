#!/usr/bin/env python3
"""Interactive Panda test tool for the STM32G474 HRR CAN controller."""

from __future__ import annotations

import argparse
import shlex
import struct
import sys
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
  from panda import Panda


TORQUE_ADDR = 0x160
BRAKE_ADDR = 0x2C6
STEER_TORQUE_SENSOR_ADDR = 0x260
STEER_TORQUE_SENSOR_BUS = 0
SVEC_CONFIG_ADDR = 0x603
ADC_STATUS_ADDR = 0x630
IO_STATUS_ADDR = 0x631
ANGLE_STATUS_ADDR = 0x632
VOLTAGE_STATUS_ADDR = 0x633
STATE_STATUS_ADDR = 0x634
TRUE_ANGLE_STATUS_ADDR = 0x636
CONFIG_STATUS_ADDR = 0x639
HRR_STATUS_ADDRS = {
  ADC_STATUS_ADDR,
  IO_STATUS_ADDR,
  ANGLE_STATUS_ADDR,
  VOLTAGE_STATUS_ADDR,
  STATE_STATUS_ADDR,
  TRUE_ANGLE_STATUS_ADDR,
  CONFIG_STATUS_ADDR,
}

DEFAULT_RATE_HZ = 100.0
DEVICE_ONLINE_TIMEOUT_S = 0.5
MAX_TORQUE_NCM = 1000
MAX_DLY_SAMPLES = 127
ANGLE_OFFSET_SCALE = 10
SVEC_ZERO_OFFSET_MAX_TENTHS_DEG = 135

FLAG_REL = 1 << 0
FLAG_RELE = 1 << 1
SVEC_CONFIG_CMD_DLY = 1
SVEC_CONFIG_CMD_PHASEMATCH = 2
SVEC_CONFIG_CMD_ANGLE_OFFSET = 3
SVEC_CONFIG_CMD_ZERO_OFFSET = 8
SVEC_CONFIG_CMD_SAVE = 9
SVEC_CONFIG_KEY = 0xA5

# Payloads observed on LS600h bus 1 for the inferred BRAKE_PRESSED state.
BRAKE_RELEASED_PAYLOAD = bytes.fromhex("99 20 84")
BRAKE_PRESSED_PAYLOAD = bytes.fromhex("9B 20 86")


def crc8_poly07(data: bytes) -> int:
  crc = 0
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


def build_torque_frame(torque_ncm: int, engaged: bool, counter: int) -> bytes:
  if not -MAX_TORQUE_NCM <= torque_ncm <= MAX_TORQUE_NCM:
    raise ValueError(f"torque must be within +/-{MAX_TORQUE_NCM} Ncm")

  torque_raw = torque_ncm & 0x0FFF
  complement = torque_raw ^ 0x0FFF
  flags = FLAG_REL | FLAG_RELE if engaged else 0
  payload = struct.pack("<HHBB", torque_raw, complement, flags, counter & 0x0F)
  checksum = crc8_poly07(bytes((TORQUE_ADDR & 0xFF, TORQUE_ADDR >> 8)) + payload)
  return payload + bytes((checksum,))


def build_brake_frame(brake_pressed: bool) -> bytes:
  return BRAKE_PRESSED_PAYLOAD if brake_pressed else BRAKE_RELEASED_PAYLOAD


def build_svec_config_frame(command: int, value: int, *, signed: bool = False) -> bytes:
  """Build a keyed, one-shot SVEC configuration command."""
  payload = bytes((command,)) + value.to_bytes(4, "little", signed=signed) + bytes((SVEC_CONFIG_KEY, 0))
  checksum = crc8_poly07(bytes((SVEC_CONFIG_ADDR & 0xFF, SVEC_CONFIG_ADDR >> 8)) + payload)
  return payload + bytes((checksum,))


def build_dly_frame(samples: int) -> bytes:
  if not 0 <= samples <= MAX_DLY_SAMPLES:
    raise ValueError(f"DLY must be within 0..{MAX_DLY_SAMPLES} samples")

  return build_svec_config_frame(SVEC_CONFIG_CMD_DLY, samples)


def build_angle_offset_frame(offset_tenths_deg: int) -> bytes:
  return build_svec_config_frame(SVEC_CONFIG_CMD_ANGLE_OFFSET, offset_tenths_deg, signed=True)


def build_svec_zero_offset_frame(offset_tenths_deg: int) -> bytes:
  if not -SVEC_ZERO_OFFSET_MAX_TENTHS_DEG <= offset_tenths_deg <= SVEC_ZERO_OFFSET_MAX_TENTHS_DEG:
    raise ValueError(
      f"SVEC_ZERO_OFFSET must be within +/-{SVEC_ZERO_OFFSET_MAX_TENTHS_DEG} tenths of a degree"
    )
  return build_svec_config_frame(SVEC_CONFIG_CMD_ZERO_OFFSET, offset_tenths_deg, signed=True)


def build_svec_save_frame() -> bytes:
  return build_svec_config_frame(SVEC_CONFIG_CMD_SAVE, 0)


def decode_io_status(payload: bytes) -> tuple[bool, bool]:
  if len(payload) < 5:
    raise ValueError(f"expected 5 bytes for CANCTR_IOStatus, got {len(payload)}")
  flags = payload[4]
  return bool(flags & FLAG_REL), bool(flags & FLAG_RELE)


def decode_angle_status(payload: bytes) -> tuple[float, int, float, float]:
  if len(payload) < 8:
    raise ValueError(f"expected 8 bytes for HRR_AngleStatus, got {len(payload)}")
  svec_delta_raw, emulated_torque, ou_angle_raw, in_angle_raw = struct.unpack_from("<hhHH", payload)
  return svec_delta_raw * 0.1, emulated_torque, ou_angle_raw * 0.1, in_angle_raw * 0.1


def decode_state_status(payload: bytes) -> float:
  if len(payload) < 4:
    raise ValueError(f"expected 4 bytes for HRR_StateStatus, got {len(payload)}")
  return int.from_bytes(payload[2:4], "little", signed=True) * 0.1


def decode_true_angle_status(payload: bytes) -> tuple[float, int, bool]:
  if len(payload) < 8:
    raise ValueError(f"expected 8 bytes for HRR_TrueAngleStatus, got {len(payload)}")
  angle = int.from_bytes(payload[0:3], "little", signed=True) * 0.01
  dly_samples = payload[3]
  if dly_samples > MAX_DLY_SAMPLES:
    raise ValueError(f"invalid reported DLY value {dly_samples}")
  return angle, dly_samples, bool(payload[6] & 0x01)


def decode_config_status(payload: bytes) -> tuple[int, int, int, bool, bool, bool, int, int]:
  if len(payload) < 8:
    raise ValueError(f"expected 8 bytes for HRR_ConfigStatus, got {len(payload)}")
  dly_samples, zero_offset, angle_offset, flags, last_command, counter = struct.unpack("<BhhBBB", payload)
  if dly_samples > MAX_DLY_SAMPLES:
    raise ValueError(f"invalid reported DLY value {dly_samples}")
  return (dly_samples, zero_offset, angle_offset, bool(flags & 0x01), bool(flags & 0x02),
          bool(flags & 0x04), last_command, counter)


def decode_steer_torque_sensor(payload: bytes) -> tuple[int, int]:
  """Decode driver and EPS torque from Toyota/Lexus STEER_TORQUE_SENSOR (0x260)."""
  if len(payload) < 7:
    raise ValueError(f"expected at least 7 bytes for STEER_TORQUE_SENSOR, got {len(payload)}")
  # These signed 16-bit Motorola fields start at DBC bits 15 and 47. In byte
  # order they occupy bytes 1..2 and 5..6 respectively; bytes 0 and 4 carry
  # preceding fields, so treating the fields as bytes 0..1 and 4..5 yields
  # values such as -30465 and +255 from otherwise sane torque feedback.
  return int.from_bytes(payload[1:3], "big", signed=True), int.from_bytes(payload[5:7], "big", signed=True)


class HrrDeviceStatus:
  def __init__(self) -> None:
    self.lock = threading.Lock()
    self.started_at = time.monotonic()
    self.last_rx_at: float | None = None
    self.rel: bool | None = None
    self.rele: bool | None = None
    self.svec_delta: float | None = None
    self.emulated_torque: int | None = None
    self.ou_angle: float | None = None
    self.in_angle: float | None = None
    self.angle_delta: float | None = None
    self.true_steering_angle: float | None = None
    self.true_angle_valid: bool | None = None
    self.dly_samples: int | None = None
    self.svec_zero_offset_tenths_deg: int | None = None
    self.angle_offset_tenths_deg: int | None = None
    self.settings_dirty: bool | None = None
    self.last_config_applied: bool | None = None
    self.last_save_succeeded: bool | None = None
    self.last_config_command: int | None = None
    self.config_counter: int | None = None

  def update(self, address: int, payload: bytes) -> bool:
    if address not in HRR_STATUS_ADDRS:
      return False

    io_status = None
    angle_status = None
    angle_delta = None
    true_angle_status = None
    config_status = None
    try:
      if address == IO_STATUS_ADDR:
        io_status = decode_io_status(payload)
      elif address == ANGLE_STATUS_ADDR:
        angle_status = decode_angle_status(payload)
      elif address == STATE_STATUS_ADDR:
        angle_delta = decode_state_status(payload)
      elif address == TRUE_ANGLE_STATUS_ADDR:
        true_angle_status = decode_true_angle_status(payload)
      elif address == CONFIG_STATUS_ADDR:
        config_status = decode_config_status(payload)
    except ValueError:
      # A recognized frame still proves that the HRR is transmitting. Leave the
      # last successfully decoded values in place when its DLC is unexpected.
      pass

    with self.lock:
      self.last_rx_at = time.monotonic()
      if io_status is not None:
        self.rel, self.rele = io_status
      if angle_status is not None:
        self.svec_delta, self.emulated_torque, self.ou_angle, self.in_angle = angle_status
      if angle_delta is not None:
        self.angle_delta = angle_delta
      if true_angle_status is not None:
        self.true_steering_angle, self.dly_samples, self.true_angle_valid = true_angle_status
      if config_status is not None:
        (self.dly_samples, self.svec_zero_offset_tenths_deg, self.angle_offset_tenths_deg,
         self.settings_dirty, self.last_config_applied, self.last_save_succeeded,
         self.last_config_command, self.config_counter) = config_status
    return True

  def reported_dly_samples(self) -> int | None:
    with self.lock:
      return self.dly_samples

  def format(self, bus: int, dry_run: bool) -> str:
    now = time.monotonic()
    with self.lock:
      last_rx_at = self.last_rx_at
      rel = self.rel
      rele = self.rele
      svec_delta = self.svec_delta
      emulated_torque = self.emulated_torque
      ou_angle = self.ou_angle
      in_angle = self.in_angle
      angle_delta = self.angle_delta
      true_steering_angle = self.true_steering_angle
      true_angle_valid = self.true_angle_valid
      dly_samples = self.dly_samples
      zero_offset = self.svec_zero_offset_tenths_deg
      angle_offset = self.angle_offset_tenths_deg
      settings_dirty = self.settings_dirty
      started_at = self.started_at

    if dry_run:
      device = "DRY-RUN"
      age = "n/a"
    elif last_rx_at is None:
      device = "WAITING" if now - started_at <= DEVICE_ONLINE_TIMEOUT_S else "OFFLINE"
      age = "never"
    else:
      rx_age = now - last_rx_at
      device = "ONLINE" if rx_age <= DEVICE_ONLINE_TIMEOUT_S else "OFFLINE"
      age = f"{rx_age:.3f}s"

    rel_text = "---" if rel is None else ("ON" if rel else "OFF")
    rele_text = "---" if rele is None else ("ON" if rele else "OFF")
    delta_text = "---.-" if svec_delta is None else f"{svec_delta:+.1f}"
    torque_text = "-----" if emulated_torque is None else f"{emulated_torque:+d}"
    ou_text = "---.-" if ou_angle is None else f"{ou_angle:.1f}"
    in_text = "---.-" if in_angle is None else f"{in_angle:.1f}"
    angle_delta_text = "---.-" if angle_delta is None else f"{angle_delta:+.1f}"
    true_angle_text = "--------" if true_steering_angle is None else f"{true_steering_angle:+.2f}"
    true_angle_valid_text = "---" if true_angle_valid is None else str(true_angle_valid)
    dly_text = "---" if dly_samples is None else str(dly_samples)
    zero_text = "---.-" if zero_offset is None else f"{zero_offset / ANGLE_OFFSET_SCALE:+.1f}"
    angle_offset_text = "---.-" if angle_offset is None else f"{angle_offset / ANGLE_OFFSET_SCALE:+.1f}"
    dirty_text = "---" if settings_dirty is None else ("YES" if settings_dirty else "NO")
    return "\n".join((
      f"RX device={device} age={age} bus={bus} REL={rel_text} RELE={rele_text}",
      f"   SVEC_Delta={delta_text}deg Emulated_Torque={torque_text}Ncm",
      f"   OU_Angle={ou_text}deg IN_Angle={in_text}deg",
      f"   Angle_Delta={angle_delta_text}deg",
      f"   True_Steering_Angle={true_angle_text}deg valid={true_angle_valid_text} DLY={dly_text} samples",
      f"   SVEC_ZERO_OFFSET={zero_text}deg ANGLE_OFFSET={angle_offset_text}deg dirty={dirty_text}",
    ))


class SteeringTorqueStatus:
  def __init__(self) -> None:
    self.lock = threading.Lock()
    self.driver_torque: int | None = None
    self.eps_torque: int | None = None

  def update(self, payload: bytes) -> bool:
    try:
      driver_torque, eps_torque = decode_steer_torque_sensor(payload)
    except ValueError:
      return False

    with self.lock:
      self.driver_torque = driver_torque
      self.eps_torque = eps_torque
    return True

  def format(self) -> str:
    with self.lock:
      driver_torque = self.driver_torque
      eps_torque = self.eps_torque
    driver_text = "-----" if driver_torque is None else f"{driver_torque:+d}"
    eps_text = "-----" if eps_torque is None else f"{eps_torque:+d}"
    return f"CAN0 0x{STEER_TORQUE_SENSOR_ADDR:03X} Driver_Torque={driver_text} EPS_Torque={eps_text}"


class StatusDisplay:
  def __init__(self, initial_message: str) -> None:
    self.lock = threading.Lock()
    self.latest_message = initial_message
    self.prompt_active = False
    self.use_ansi = sys.stdin.isatty() and sys.stdout.isatty()

  def show_prompt_header(self) -> None:
    with self.lock:
      print(self.latest_message)

  def set_prompt_active(self, active: bool) -> None:
    with self.lock:
      self.prompt_active = active

  def update(self, message: str) -> None:
    with self.lock:
      self.latest_message = message
      if self.use_ansi and self.prompt_active:
        lines = message.splitlines()
        sys.stdout.write(f"\x1b7\x1b[{len(lines)}A")
        for index, line in enumerate(lines):
          sys.stdout.write(f"\r\x1b[2K{line}")
          if index != len(lines) - 1:
            sys.stdout.write("\x1b[1B")
        sys.stdout.write("\x1b8")
        sys.stdout.flush()


class CommandState:
  def __init__(self) -> None:
    self.lock = threading.Lock()
    self.engaged = False
    self.torque_ncm = 0
    self.brake_pressed = False
    self.counter = 0
    self.dly_samples: int | None = None
    self.angle_offset_tenths_deg: int | None = None
    self.svec_zero_offset_tenths_deg: int | None = None

  def next_frames(self) -> tuple[bytes, bytes]:
    with self.lock:
      brake = build_brake_frame(self.brake_pressed)
      torque = build_torque_frame(self.torque_ncm, self.engaged, self.counter)
      self.counter = (self.counter + 1) & 0x0F
      return brake, torque

  def set_engaged(self, engaged: bool) -> None:
    with self.lock:
      self.engaged = engaged
      if not engaged:
        self.torque_ncm = 0

  def set_torque(self, torque_ncm: int) -> None:
    if not -MAX_TORQUE_NCM <= torque_ncm <= MAX_TORQUE_NCM:
      raise ValueError(f"torque must be within +/-{MAX_TORQUE_NCM} Ncm")
    with self.lock:
      self.torque_ncm = torque_ncm

  def set_brake(self, brake_pressed: bool) -> None:
    with self.lock:
      self.brake_pressed = brake_pressed

  def set_dly(self, samples: int) -> None:
    if not 0 <= samples <= MAX_DLY_SAMPLES:
      raise ValueError(f"DLY must be within 0..{MAX_DLY_SAMPLES} samples")
    with self.lock:
      self.dly_samples = samples

  def set_angle_offset(self, offset_tenths_deg: int) -> None:
    with self.lock:
      self.angle_offset_tenths_deg = offset_tenths_deg

  def set_svec_zero_offset(self, offset_tenths_deg: int) -> None:
    if not -SVEC_ZERO_OFFSET_MAX_TENTHS_DEG <= offset_tenths_deg <= SVEC_ZERO_OFFSET_MAX_TENTHS_DEG:
      raise ValueError(
        f"SVEC_ZERO_OFFSET must be within +/-{SVEC_ZERO_OFFSET_MAX_TENTHS_DEG} tenths of a degree"
      )
    with self.lock:
      self.svec_zero_offset_tenths_deg = offset_tenths_deg

  def snapshot(self) -> tuple[bool, int, bool, int | None, int | None, int | None]:
    with self.lock:
      return (self.engaged, self.torque_ncm, self.brake_pressed, self.dly_samples,
              self.angle_offset_tenths_deg, self.svec_zero_offset_tenths_deg)


class HrrCanTest:
  def __init__(self, panda: Panda | None, bus: int, rate_hz: float, dry_run: bool) -> None:
    self.panda = panda
    self.bus = bus
    self.period = 1.0 / rate_hz
    self.dry_run = dry_run
    self.state = CommandState()
    self.device_status = HrrDeviceStatus()
    self.steering_torque_status = SteeringTorqueStatus()
    self.status_display = StatusDisplay(self.format_status())
    self.stop_event = threading.Event()
    self.stream_thread: threading.Thread | None = None
    self.monitor_thread: threading.Thread | None = None
    self.harness_relay_forced = False
    self.harness_relay_controlled = False

  def format_status(self) -> str:
    return "\n".join((
      self.device_status.format(self.bus, self.dry_run),
      self.steering_torque_status.format(),
    ))

  def send(self, address: int, payload: bytes) -> None:
    if self.dry_run:
      return
    assert self.panda is not None
    self.panda.can_send(address, payload, self.bus)

  def send_stream_frames(self) -> None:
    brake, torque = self.state.next_frames()
    # Brake goes first so the HRR interlock is fresh before each torque command.
    self.send(BRAKE_ADDR, brake)
    self.send(TORQUE_ADDR, torque)

  def set_force_harness_relay(self, enabled: bool) -> None:
    self.harness_relay_controlled = True
    if self.panda is not None:
      # Match the ForceHarnessRelayOn UI behavior: keep CAN0/CAN2 physically
      # separated and prevent panda firmware from forwarding between them.
      self.panda.set_force_intercept_relay(enabled)
      self.panda.set_safety_forwarding_disabled(enabled)
    self.harness_relay_forced = enabled

  def restore_harness_relay(self) -> None:
    if not self.harness_relay_controlled:
      return
    if self.panda is not None:
      self.panda.set_force_intercept_relay(False)
      self.panda.set_safety_forwarding_disabled(False)
    self.harness_relay_forced = False

  def stream_loop(self) -> None:
    next_send = time.monotonic()
    while not self.stop_event.is_set():
      self.send_stream_frames()
      next_send += self.period
      wait = next_send - time.monotonic()
      if wait < 0:
        next_send = time.monotonic()
        continue
      self.stop_event.wait(wait)

  def monitor_loop(self) -> None:
    assert self.panda is not None
    next_display = time.monotonic()
    while not self.stop_event.is_set():
      for address, _, payload, rx_bus in self.panda.can_recv():
        if rx_bus == self.bus:
          self.device_status.update(address, payload)
        if rx_bus == STEER_TORQUE_SENSOR_BUS and address == STEER_TORQUE_SENSOR_ADDR:
          self.steering_torque_status.update(payload)

      now = time.monotonic()
      if now >= next_display:
        self.status_display.update(self.format_status())
        next_display = now + 0.1
      self.stop_event.wait(0.01)

  def start(self) -> None:
    if self.panda is not None:
      self.panda.can_clear(0xFFFF)
      self.monitor_thread = threading.Thread(target=self.monitor_loop, name="hrr-can-monitor", daemon=True)
      self.monitor_thread.start()
    self.stream_thread = threading.Thread(target=self.stream_loop, name="hrr-can-stream", daemon=True)
    self.stream_thread.start()

  def send_dly(self, samples: int) -> None:
    payload = build_dly_frame(samples)
    self.send(SVEC_CONFIG_ADDR, payload)
    self.state.set_dly(samples)
    print(f"DLY={samples} samples sent on 0x{SVEC_CONFIG_ADDR:03X}: {payload.hex(' ')}")
    print("DLY is active but not persistent until the settings are saved.")

  def send_angle_offset(self, offset_tenths_deg: int) -> None:
    payload = build_angle_offset_frame(offset_tenths_deg)
    self.send(SVEC_CONFIG_ADDR, payload)
    self.state.set_angle_offset(offset_tenths_deg)
    print(
      f"ANGLE_OFFSET={offset_tenths_deg / ANGLE_OFFSET_SCALE:+.1f} deg sent on "
      + f"0x{SVEC_CONFIG_ADDR:03X}: {payload.hex(' ')}"
    )
    print("ANGLE_OFFSET is active but not persistent until the settings are saved.")

  def send_svec_zero_offset(self, offset_tenths_deg: int) -> None:
    payload = build_svec_zero_offset_frame(offset_tenths_deg)
    self.send(SVEC_CONFIG_ADDR, payload)
    self.state.set_svec_zero_offset(offset_tenths_deg)
    print(
      f"SVEC_ZERO_OFFSET={offset_tenths_deg / ANGLE_OFFSET_SCALE:+.1f} deg sent on "
      + f"0x{SVEC_CONFIG_ADDR:03X}: {payload.hex(' ')}"
    )
    print("SVEC_ZERO_OFFSET is active but not persistent until the settings are saved.")

  def save_svec_settings(self) -> None:
    payload = build_svec_save_frame()
    self.send(SVEC_CONFIG_ADDR, payload)
    print(f"Save-current-settings command sent on 0x{SVEC_CONFIG_ADDR:03X}: {payload.hex(' ')}")
    print("Check HRR_ConfigStatus (0x639) for dirty=NO and save success.")

  def safe_shutdown(self) -> None:
    self.state.set_engaged(False)
    self.state.set_brake(True)
    for _ in range(5):
      self.send_stream_frames()
      time.sleep(0.01)
    self.stop_event.set()
    if self.stream_thread is not None:
      self.stream_thread.join(timeout=1.0)
    if self.monitor_thread is not None:
      self.monitor_thread.join(timeout=1.0)

  def print_state(self) -> None:
    (engaged, torque_ncm, brake_pressed, dly_samples, angle_offset_tenths_deg,
     svec_zero_offset_tenths_deg) = self.state.snapshot()
    reported_dly_samples = self.device_status.reported_dly_samples()
    if reported_dly_samples is not None:
      dly_samples = reported_dly_samples
    with self.device_status.lock:
      if self.device_status.angle_offset_tenths_deg is not None:
        angle_offset_tenths_deg = self.device_status.angle_offset_tenths_deg
      if self.device_status.svec_zero_offset_tenths_deg is not None:
        svec_zero_offset_tenths_deg = self.device_status.svec_zero_offset_tenths_deg
      settings_dirty = self.device_status.settings_dirty
    dly_text = "waiting for 0x636" if dly_samples is None else f"{dly_samples} samples"
    angle_offset_text = ("unknown (not reported)" if angle_offset_tenths_deg is None
                         else f"{angle_offset_tenths_deg / ANGLE_OFFSET_SCALE:+.1f} deg")
    svec_zero_offset_text = ("unknown (not reported)" if svec_zero_offset_tenths_deg is None
                             else f"{svec_zero_offset_tenths_deg / ANGLE_OFFSET_SCALE:+.1f} deg")
    print(" ".join((
      f"TX bus={self.bus} engaged={engaged} REL_Cmd={int(engaged)} RELE_Cmd={int(engaged)}",
      f"torque={torque_ncm:+d} Ncm brake_pressed={brake_pressed}",
      f"DLY={dly_text} ANGLE_OFFSET={angle_offset_text} SVEC_ZERO_OFFSET={svec_zero_offset_text}",
      f"settings_dirty={'unknown' if settings_dirty is None else settings_dirty}",
      f"harness_relay={'FORCED' if self.harness_relay_forced else 'AUTO'}",
    )))


def parse_on_off(value: str) -> bool:
  value = value.lower()
  if value in {"1", "on", "true", "pressed", "press"}:
    return True
  if value in {"0", "off", "false", "released", "release"}:
    return False
  raise ValueError("expected on/off, 1/0, pressed/released, or true/false")


def print_help() -> None:
  print("Commands:")
  print("  e                       engage: REL and RELE on")
  print("  x                       disengage: torque 0, REL and RELE off")
  print(f"  <Ncm>                   set torque directly, range +/-{MAX_TORQUE_NCM}")
  print(f"  d <samples>             set active SVEC DLY, range 0..{MAX_DLY_SAMPLES}")
  print("  a <tenths-deg>          set active OU-IN diagnostic/guard ANGLE_OFFSET")
  print("                            (does not change SVEC output or Driver_Torque)")
  print(f"  z <tenths-deg>          set active SVEC_ZERO_OFFSET, range +/-{SVEC_ZERO_OFFSET_MAX_TENTHS_DEG}")
  print("                            (actuation bias added only while REL and RELE are on)")
  print("  b <0|1>                 BRAKE_PRESSED: 0=released, 1=pressed")
  print("  w                       persist the current DLY and both offsets")
  print("  r <0|1>                 force harness relay and disable forwarding")
  print("  s                       show current state")
  print("  h                       show this help")
  print("  q                       safe shutdown and exit")
  print("ANGLE_OFFSET corrects only reported Angle_Delta and the optional SVEC guard.")
  print("SVEC_ZERO_OFFSET shifts the center of the full +/-13.5 deg torque span; effective range is up to +/-27.0 deg.")


def choose_bus(configured_bus: int | None) -> int:
  if configured_bus is not None:
    return configured_bus
  while True:
    value = input("Panda CAN bus [0/1/2]: ").strip()
    if value in {"0", "1", "2"}:
      return int(value)
    print("Enter 0, 1, or 2.")


def run_self_test() -> None:
  assert build_torque_frame(0, False, 0).hex() == "0000ff0f0000fb"
  assert build_dly_frame(27).hex() == "011b000000a500cd"
  assert build_angle_offset_frame(-45).hex() == "03d3ffffffa50062"
  assert build_svec_zero_offset_frame(-15).hex() == "08f1ffffffa50087"
  assert build_svec_save_frame().hex() == "0900000000a5008d"
  for invalid_zero_offset in (-136, 136):
    try:
      build_svec_zero_offset_frame(invalid_zero_offset)
    except ValueError:
      pass
    else:
      raise AssertionError("out-of-range SVEC_ZERO_OFFSET was accepted")
  assert build_brake_frame(False) == bytes.fromhex("99 20 84")
  assert build_brake_frame(True)[0] & 0x02
  assert decode_io_status(bytes.fromhex("00 08 ff 07 03")) == (True, True)
  angle_payload = struct.pack("<hhHH", -40, -250, 1234, 567)
  assert decode_angle_status(angle_payload) == (-4.0, -250, 123.4, 56.7)
  assert decode_state_status(bytes.fromhex("00 00 d3 ff 00 00 00 00")) == -4.5
  true_angle_payload = (
    (-12345 & 0xFFFFFF).to_bytes(3, "little")
    + bytes((42,))
    + struct.pack("<hBB", 123, 0x1F, 7)
  )
  true_angle, reported_dly, true_angle_valid = decode_true_angle_status(true_angle_payload)
  assert abs(true_angle - (-123.45)) < 1e-9
  assert reported_dly == 42
  assert true_angle_valid
  config_payload = struct.pack("<BhhBBB", 42, -15, -45, 0x07, SVEC_CONFIG_CMD_SAVE, 9)
  assert decode_config_status(config_payload) == (42, -15, -45, True, True, True, 9, 9)
  assert decode_steer_torque_sensor(bytes.fromhex("00 ff 06 00 00 fe d4 00")) == (-250, -300)

  device_status = HrrDeviceStatus()
  assert device_status.update(IO_STATUS_ADDR, bytes.fromhex("00 08 ff 07 03"))
  assert device_status.update(ANGLE_STATUS_ADDR, angle_payload)
  assert device_status.update(STATE_STATUS_ADDR, bytes.fromhex("00 00 2d 00 00 00 00 00"))
  assert device_status.update(TRUE_ANGLE_STATUS_ADDR, true_angle_payload)
  assert device_status.update(CONFIG_STATUS_ADDR, config_payload)
  status_text = device_status.format(1, False)
  for expected in ("device=ONLINE", "REL=ON", "RELE=ON", "SVEC_Delta=-4.0deg",
                   "Emulated_Torque=-250Ncm", "OU_Angle=123.4deg", "IN_Angle=56.7deg",
                   "True_Steering_Angle=-123.45deg", "valid=True", "DLY=42 samples"):
    assert expected in status_text
  assert "Angle_Delta=+4.5deg" in status_text
  assert "SVEC_ZERO_OFFSET=-1.5deg" in status_text
  assert "ANGLE_OFFSET=-4.5deg" in status_text
  assert "dirty=YES" in status_text

  steering_torque_status = SteeringTorqueStatus()
  assert steering_torque_status.update(bytes.fromhex("00 00 fa 00 00 ff 06 00"))
  assert "Driver_Torque=+250" in steering_torque_status.format()
  assert "EPS_Torque=-250" in steering_torque_status.format()

  class FakePanda:
    def __init__(self) -> None:
      self.calls: list[tuple[str, bool]] = []

    def set_force_intercept_relay(self, enabled: bool) -> None:
      self.calls.append(("relay", enabled))

    def set_safety_forwarding_disabled(self, disabled: bool) -> None:
      self.calls.append(("forwarding_disabled", disabled))

  fake_panda = FakePanda()
  test = HrrCanTest(fake_panda, 1, DEFAULT_RATE_HZ, False)  # type: ignore[arg-type]
  test.set_force_harness_relay(True)
  test.restore_harness_relay()
  assert fake_panda.calls == [
    ("relay", True), ("forwarding_disabled", True),
    ("relay", False), ("forwarding_disabled", False),
  ]
  print("Frame self-test passed.")


def run_interactive(test: HrrCanTest) -> None:
  print_help()
  test.print_state()
  while True:
    try:
      test.status_display.show_prompt_header()
      test.status_display.set_prompt_active(True)
      try:
        tokens = shlex.split(input("hrr> ").strip())
      finally:
        test.status_display.set_prompt_active(False)
      if not tokens:
        continue
      command = tokens[0].lower()

      if command in {"q", "quit", "exit"}:
        return
      if command in {"h", "help", "?"}:
        print_help()
      elif command in {"s", "show", "status"}:
        test.print_state()
      elif command in {"e", "engage"} and len(tokens) == 1:
        test.state.set_engaged(True)
        test.print_state()
      elif command in {"x", "disengage"} and len(tokens) == 1:
        test.state.set_engaged(False)
        test.print_state()
      elif command in {"torque", "tq"} and len(tokens) == 2:
        test.state.set_torque(int(tokens[1], 0))
        test.print_state()
      elif command in {"d", "dly"} and len(tokens) == 2:
        test.send_dly(int(tokens[1], 0))
      elif command in {"a", "angle_offset"} and len(tokens) == 2:
        test.send_angle_offset(int(tokens[1], 0))
      elif command in {"z", "zero_offset", "svec_zero_offset"} and len(tokens) == 2:
        test.send_svec_zero_offset(int(tokens[1], 0))
      elif command in {"w", "write", "save"} and len(tokens) == 1:
        test.save_svec_settings()
      elif command in {"b", "brake", "brake_pressed"} and len(tokens) == 2:
        test.state.set_brake(parse_on_off(tokens[1]))
        test.print_state()
      elif command in {"r", "relay", "harness_relay"} and len(tokens) == 2:
        test.set_force_harness_relay(parse_on_off(tokens[1]))
        test.print_state()
      elif len(tokens) == 1:
        test.state.set_torque(int(tokens[0], 0))
        test.print_state()
      else:
        print("Unknown or malformed command. Enter 'h'.")
    except ValueError as error:
      print(f"Invalid command: {error}")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("--bus", type=int, choices=(0, 1, 2), help="Panda CAN bus; prompted when omitted")
  parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ, help="torque and brake streaming rate")
  parser.add_argument("--force-harness-relay", action="store_true",
                      help="force the Panda harness relay and disable firmware forwarding while the tool runs")
  parser.add_argument("--dry-run", action="store_true", help="exercise controls without opening or transmitting through Panda")
  parser.add_argument("--self-test", action="store_true", help="verify known frame encodings and exit")
  args = parser.parse_args()

  if args.self_test:
    run_self_test()
    return
  if args.rate_hz <= 0:
    parser.error("--rate-hz must be greater than zero")

  bus = choose_bus(args.bus)
  panda = None
  if not args.dry_run:
    from panda import Panda
    panda = Panda()
  if panda is not None:
    panda.set_power_save(False)
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  test = HrrCanTest(panda, bus, args.rate_hz, args.dry_run)
  try:
    if args.force_harness_relay:
      test.set_force_harness_relay(True)
    print(f"Streaming 0x{TORQUE_ADDR:03X} and 0x{BRAKE_ADDR:03X} on Panda bus {bus} at {args.rate_hz:g} Hz.")
    print("Initial state is disengaged, zero torque, and brake released.")
    test.start()
    run_interactive(test)
  except (KeyboardInterrupt, EOFError):
    print()
  finally:
    print("Safe shutdown: torque=0, relays off, BRAKE_PRESSED=1.")
    test.safe_shutdown()
    if panda is not None:
      panda.set_safety_mode(Panda.SAFETY_SILENT)
    test.restore_harness_relay()


if __name__ == "__main__":
  main()
