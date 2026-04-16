#!/usr/bin/env python3
import argparse
import struct
import sys
import threading
import time

from panda import Panda

CANCTR_DELTA_ADDR = 0x231
VSM2_ADDR = 0x165
CANCTR_IO_STATUS_ADDR = 0x241
CANCTR_VOLTAGE_STATUS_ADDR = 0x243
DEFAULT_BUS = 1
TORQUE_MONITOR_BUS = 0
VOLTAGE_MONITOR_BUS = 1
DAC_BITS = 12
DAC_FULL_SCALE_VOLTS = 5.0
DAC_MAX_CODE = (1 << DAC_BITS) - 1
MAX_DELTA = 2047
INPUT_SCALE = 1000.0
TORQUE_REFERENCE = 100.0
LEGACY_INPUT_AT_TORQUE_REFERENCE = 165.0
VOLTAGE_STATUS_SCALE = 0.00125
DEFAULT_KEEPALIVE_PERIOD_MS = 100.0
CANCTR_WATCHDOG_TIMEOUT_MS = 500.0

FLAG_REL = 1 << 0
FLAG_RELE = 1 << 1

IO_STATUS_FLAG_REL = 1 << 0
IO_STATUS_FLAG_RELE = 1 << 1
IO_STATUS_FLAG_RC_MINUS = 1 << 2
IO_STATUS_FLAG_RC_PLUS = 1 << 3
IO_STATUS_FLAG_DELTA_DLC_ERROR = 1 << 4
IO_STATUS_FLAG_DELTA_MISMATCH_ERROR = 1 << 5
IO_STATUS_FLAG_DELTA_CHECKSUM_ERROR = 1 << 6
IO_STATUS_FLAG_WATCHDOG_TIMEOUT = 1 << 7

IO_STATUS_AUX_SNR_PAIR_ERROR = 1 << 0
IO_STATUS_AUX_EPS_PAIR_ERROR = 1 << 1
IO_STATUS_AUX_DELTA_COUNTER_ERROR = 1 << 2
IO_STATUS_AUX_OUTPUT_RANGE_ERROR = 1 << 7

EXIT_WORDS = {"q", "quit", "exit"}


class MonitorDisplay:
  def __init__(self, torque_bus: int, voltage_bus: int, interactive: bool) -> None:
    self.torque_bus = torque_bus
    self.voltage_bus = voltage_bus
    self.interactive = interactive
    self.use_ansi = sys.stdout.isatty()
    self.lock = threading.Lock()
    self.prompt_active = False
    self.latest_message = (
      f"Monitoring VSM2 on bus {torque_bus} and VIN status on bus {voltage_bus}. Waiting for torque and voltage data..."
    )

  def show_prompt_header(self) -> None:
    with self.lock:
      print(self.latest_message)

  def set_prompt_active(self, active: bool) -> None:
    with self.lock:
      self.prompt_active = active

  def update(self, message: str) -> None:
    with self.lock:
      self.latest_message = message

      if self.interactive:
        if self.use_ansi and self.prompt_active:
          sys.stdout.write(f"\x1b7\x1b[1A\r\x1b[2K{message}\x1b8")
          sys.stdout.flush()
        elif not self.use_ansi:
          print(message)
        return

      if self.use_ansi:
        sys.stdout.write(f"\r\x1b[2K{message}")
        sys.stdout.flush()
      else:
        print(message)

  def finish(self) -> None:
    if not self.use_ansi:
      return

    with self.lock:
      sys.stdout.write("\n")
      sys.stdout.flush()


class CanctrStatusCache:
  def __init__(self) -> None:
    self.lock = threading.Lock()
    self.latest_io_status = None

  def update_io_status(self, io_status):
    with self.lock:
      self.latest_io_status = io_status

  def get_io_status(self):
    with self.lock:
      if self.latest_io_status is None:
        return None
      return dict(self.latest_io_status)


class CommandState:
  def __init__(self, rel: int, rele: int, command_value: float, torque_mode: bool) -> None:
    self.lock = threading.Lock()
    self.rel = rel
    self.rele = rele
    self.command_value = command_value
    self.torque_mode = torque_mode
    self.tx_counter = 0

  def snapshot(self) -> tuple[int, int, float, bool]:
    with self.lock:
      return self.rel, self.rele, self.command_value, self.torque_mode

  def update(self, *, rel: int | None = None, rele: int | None = None,
             command_value: float | None = None) -> tuple[int, int, float, bool]:
    with self.lock:
      if rel is not None:
        self.rel = rel
      if rele is not None:
        self.rele = rele
      if command_value is not None:
        self.command_value = command_value
      return self.rel, self.rele, self.command_value, self.torque_mode

  def next_frame_params(self) -> tuple[int, int, float, bool, int]:
    with self.lock:
      counter = self.tx_counter
      self.tx_counter = (self.tx_counter + 1) & 0x0F
      return self.rel, self.rele, self.command_value, self.torque_mode, counter


def parse_bit(value: str) -> int:
  normalized = value.strip().lower()
  if normalized in {"0", "false", "f", "no", "n", "off"}:
    return 0
  if normalized in {"1", "true", "t", "yes", "y", "on"}:
    return 1
  raise ValueError(f"expected 0/1, true/false, yes/no, or on/off, got {value!r}")


def clamp_delta(delta: int) -> tuple[int, bool]:
  clamped = max(-MAX_DELTA, min(MAX_DELTA, delta))
  return clamped, clamped != delta


def voltage_to_delta(voltage: float) -> tuple[int, bool]:
  raw_delta = int(round(voltage * DAC_MAX_CODE / DAC_FULL_SCALE_VOLTS))
  return clamp_delta(raw_delta)


def delta_to_voltage(delta: int) -> float:
  return delta * DAC_FULL_SCALE_VOLTS / DAC_MAX_CODE


def input_to_voltage(command_input: float) -> float:
  return -command_input / INPUT_SCALE


def voltage_to_input(voltage: float) -> float:
  return -voltage * INPUT_SCALE


def torque_to_input(command_torque: float) -> float:
  return command_torque * LEGACY_INPUT_AT_TORQUE_REFERENCE / TORQUE_REFERENCE


def input_to_torque(command_input: float) -> float:
  return command_input * TORQUE_REFERENCE / LEGACY_INPUT_AT_TORQUE_REFERENCE


def torque_to_voltage(command_torque: float) -> float:
  return input_to_voltage(torque_to_input(command_torque))


def voltage_to_torque(voltage: float) -> float:
  return input_to_torque(voltage_to_input(voltage))


def command_to_voltage(command_value: float, torque_mode: bool) -> float:
  return torque_to_voltage(command_value) if torque_mode else input_to_voltage(command_value)


def voltage_to_command(voltage: float, torque_mode: bool) -> float:
  return voltage_to_torque(voltage) if torque_mode else voltage_to_input(voltage)


def mode_label(torque_mode: bool) -> str:
  return "torque" if torque_mode else "input"


def mode_prompt(torque_mode: bool) -> str:
  return "tq> " if torque_mode else "input> "


def compute_checksum(addr: int, payload_without_checksum: bytes) -> int:
  total = (addr & 0xFF) + ((addr >> 8) & 0xFF) + sum(payload_without_checksum)
  return total & 0xFF


def unpack_packed_12bit(payload: bytes, count: int) -> tuple[int, ...]:
  packed = int.from_bytes(payload, "little")
  return tuple((packed >> (12 * i)) & 0xFFF for i in range(count))


def build_command(delta: int, rel: int, rele: int, counter: int) -> tuple[bytes, int, int]:
  flags = (FLAG_REL if rel else 0) | (FLAG_RELE if rele else 0)
  counter_byte = counter & 0x0F
  payload_without_checksum = struct.pack("<hhBB", delta, delta, flags, counter_byte)
  checksum = compute_checksum(CANCTR_DELTA_ADDR, payload_without_checksum)
  return payload_without_checksum + bytes([checksum]), flags, checksum


def enable_output(panda: Panda) -> None:
  panda.set_power_save(0)
  panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)


def send_command(panda: Panda, bus: int, rel: int, rele: int, command_value: float,
                 torque_mode: bool, counter: int, quiet: bool = False) -> None:
  enable_output(panda)

  requested_voltage = command_to_voltage(command_value, torque_mode)
  delta, saturated = voltage_to_delta(requested_voltage)
  applied_voltage = delta_to_voltage(delta)
  applied_command_value = voltage_to_command(applied_voltage, torque_mode)
  applied_input = voltage_to_input(applied_voltage)
  payload, flags, checksum = build_command(delta, rel, rele, counter)

  panda.can_send(CANCTR_DELTA_ADDR, payload, bus)

  if quiet:
    return

  print(f"TX bus={bus} addr=0x{CANCTR_DELTA_ADDR:03X} payload={payload.hex()} flags=0x{flags:02X} counter={counter} checksum=0x{checksum:02X}")
  print(f"  {mode_label(torque_mode)} command: {command_value:+.6f}")
  if torque_mode:
    print(
      f"  applied conversion: {requested_voltage:+.6f} V using {TORQUE_REFERENCE:.0f} torque = "
      + f"{LEGACY_INPUT_AT_TORQUE_REFERENCE:.0f} legacy input"
    )
    print(f"  equivalent legacy input: {voltage_to_input(requested_voltage):+.6f}")
  else:
    print(f"  applied conversion: {requested_voltage:+.6f} V = -input/{INPUT_SCALE:.0f}")
  if saturated:
    print(
      f"  clamped to firmware range: {applied_voltage:+.6f} V ({delta:+d} counts, "
      + f"equivalent {mode_label(torque_mode)} {applied_command_value:+.6f})"
    )
  else:
    print(f"  delta: {delta:+d} counts ({applied_voltage:+.6f} V from current centerpoint)")
  if torque_mode:
    print(f"  equivalent legacy input after clamp: {applied_input:+.6f}")
  print("  DAC1 = center - delta, DAC2 = center + delta")
  print(f"  REL={rel} RELE={rele}")


def send_keepalive_loop(panda: Panda, bus: int, command_state: CommandState,
                        period_s: float, stop_event: threading.Event) -> None:
  next_send = time.monotonic() + period_s
  while not stop_event.is_set():
    wait_s = max(0.0, next_send - time.monotonic())
    if stop_event.wait(wait_s):
      break
    send_command(panda, bus, *command_state.next_frame_params(), quiet=True)
    next_send += period_s


def start_keepalive_thread(panda: Panda, bus: int, command_state: CommandState,
                           period_s: float, stop_event: threading.Event) -> threading.Thread:
  thread = threading.Thread(
    target=send_keepalive_loop,
    args=(panda, bus, command_state, period_s, stop_event),
    daemon=True,
  )
  thread.start()
  return thread


def decode_vsm2_torques(payload: bytes) -> tuple[float, float]:
  if len(payload) < 3:
    raise ValueError(f"expected at least 3 bytes for VSM2, got {len(payload)}")

  raw = int.from_bytes(payload[:3], "little")
  steering_torque = (raw & 0xFFF) * 0.01 - 20.48
  output_torque = ((raw >> 12) & 0xFFF) * 0.1 - 204.8
  return steering_torque, output_torque


def decode_voltage_status(payload: bytes) -> tuple[float, float, float, float]:
  if len(payload) < 8:
    raise ValueError(f"expected 8 bytes for CANCTR_VoltageStatus, got {len(payload)}")

  snr1_raw, snr2_raw, eps1_raw, eps2_raw = unpack_packed_12bit(payload[:6], 4)
  return (
    snr1_raw * VOLTAGE_STATUS_SCALE,
    snr2_raw * VOLTAGE_STATUS_SCALE,
    eps1_raw * VOLTAGE_STATUS_SCALE,
    eps2_raw * VOLTAGE_STATUS_SCALE,
  )


def decode_io_status(payload: bytes):
  if len(payload) < 8:
    raise ValueError(f"expected 8 bytes for CANCTR_IOStatus, got {len(payload)}")

  dac1_code, dac2_code = unpack_packed_12bit(payload[:3], 2)
  ref_voltage = int.from_bytes(payload[3:5], "little") * 0.001
  status = payload[5]
  aux = payload[6]
  checksum = payload[7]
  expected_checksum = compute_checksum(CANCTR_IO_STATUS_ADDR, payload[:7])
  return {
    "timestamp": time.monotonic(),
    "dac1_code": dac1_code,
    "dac2_code": dac2_code,
    "rel_state": bool(status & IO_STATUS_FLAG_REL),
    "rele_state": bool(status & IO_STATUS_FLAG_RELE),
    "rc_minus": bool(status & IO_STATUS_FLAG_RC_MINUS),
    "rc_plus": bool(status & IO_STATUS_FLAG_RC_PLUS),
    "delta_dlc_error": bool(status & IO_STATUS_FLAG_DELTA_DLC_ERROR),
    "delta_mismatch_error": bool(status & IO_STATUS_FLAG_DELTA_MISMATCH_ERROR),
    "delta_checksum_error": bool(status & IO_STATUS_FLAG_DELTA_CHECKSUM_ERROR),
    "watchdog_timeout": bool(status & IO_STATUS_FLAG_WATCHDOG_TIMEOUT),
    "io_status_counter": aux & 0x0F,
    "snr_pair_error": bool(aux & IO_STATUS_AUX_SNR_PAIR_ERROR),
    "eps_pair_error": bool(aux & IO_STATUS_AUX_EPS_PAIR_ERROR),
    "delta_counter_error": bool(aux & IO_STATUS_AUX_DELTA_COUNTER_ERROR),
    "output_range_error": bool(aux & IO_STATUS_AUX_OUTPUT_RANGE_ERROR),
    "eps_refv": ref_voltage,
    "checksum_ok": checksum == expected_checksum,
    "checksum": checksum,
    "expected_checksum": expected_checksum,
  }


def decode_io_status_ref_voltage(payload: bytes) -> float:
  return float(decode_io_status(payload)["eps_refv"])


def format_canctr_io_status(io_status) -> str:
  age_s = time.monotonic() - io_status["timestamp"]
  error_names = []
  if io_status["delta_dlc_error"]:
    error_names.append("DELTA_DLC_ERR")
  if io_status["delta_mismatch_error"]:
    error_names.append("DELTA_MISMATCH")
  if io_status["delta_checksum_error"]:
    error_names.append("DELTA_CHECKSUM_ERR")
  if io_status["delta_counter_error"]:
    error_names.append("DELTA_COUNTER_ERR")
  if io_status["watchdog_timeout"]:
    error_names.append("WATCHDOG_TIMEOUT")
  if io_status["snr_pair_error"]:
    error_names.append("SNR_PAIR_ERR")
  if io_status["eps_pair_error"]:
    error_names.append("EPS_PAIR_ERR")
  if io_status["output_range_error"]:
    error_names.append("OUTPUT_RANGE_ERR")

  checksum_text = "OK" if io_status["checksum_ok"] else (
    f"BAD got=0x{io_status['checksum']:02X} exp=0x{io_status['expected_checksum']:02X}"
  )
  errors_text = ", ".join(error_names) if error_names else "none"
  return (
    f"CANCTR_IOStatus age={age_s:0.3f}s checksum={checksum_text} ctr={io_status['io_status_counter']} "
    + f"DAC1={io_status['dac1_code']} DAC2={io_status['dac2_code']} EPS_REFV={io_status['eps_refv']:.3f}V "
    + f"REL={int(io_status['rel_state'])} RELE={int(io_status['rele_state'])} "
    + f"RC-={int(io_status['rc_minus'])} RC+={int(io_status['rc_plus'])} errors={errors_text}"
  )


def poll_canctr_io_status(panda: Panda, bus: int, timeout_s: float = 0.35):
  deadline = time.monotonic() + timeout_s
  while time.monotonic() < deadline:
    saw_frame = False
    for addr, _, payload, rx_bus in panda.can_recv():
      saw_frame = True
      if addr == CANCTR_IO_STATUS_ADDR and rx_bus == bus:
        return decode_io_status(payload)
    if not saw_frame:
      time.sleep(0.01)
  return None


def print_canctr_io_status(panda: Panda, status_bus: int, status_cache: CanctrStatusCache | None = None) -> None:
  io_status = status_cache.get_io_status() if status_cache is not None else None
  if io_status is None and status_cache is None:
    io_status = poll_canctr_io_status(panda, status_bus)
  if io_status is None:
    print(f"No CANCTR_IOStatus received yet on bus {status_bus}.")
    return
  print(format_canctr_io_status(io_status))


def format_optional_voltage(value: float | None) -> str:
  return f"{value:5.3f}V" if value is not None else "---.--V"


def build_monitor_message(torque_bus: int, voltage_bus: int, elapsed: float,
                          steering_torque: float | None, output_torque: float | None,
                          snr1_voltage: float | None, snr2_voltage: float | None,
                          eps1_voltage: float | None, eps2_voltage: float | None,
                          ref_voltage: float | None) -> str:
  steering_text = f"{steering_torque:+6.2f}Nm" if steering_torque is not None else "---.--Nm"
  output_text = f"{output_torque:+6.1f}" if output_torque is not None else "----.-"
  return (
    f"RX {elapsed:8.3f}s tq_bus={torque_bus} vin_bus={voltage_bus} "
    + f"StrTq={steering_text} OutTq={output_text} "
    + f"EPS1={format_optional_voltage(eps1_voltage)} "
    + f"EPS2={format_optional_voltage(eps2_voltage)} "
    + f"SNR1={format_optional_voltage(snr1_voltage)} "
    + f"SNR2={format_optional_voltage(snr2_voltage)} "
    + f"EPS_REFV={format_optional_voltage(ref_voltage)}"
  )


def monitor_vsm2(panda: Panda, torque_bus: int, voltage_bus: int,
                 display: MonitorDisplay | None = None,
                 status_cache: CanctrStatusCache | None = None) -> None:
  panda.can_clear(0xFFFF)
  start = time.monotonic()
  steering_torque = None
  output_torque = None
  snr1_voltage = None
  snr2_voltage = None
  eps1_voltage = None
  eps2_voltage = None
  ref_voltage = None

  if display is None:
    print(
      f"Monitoring VSM2 (0x{VSM2_ADDR:03X}) on bus {torque_bus}, CANCTR_VoltageStatus "
      + f"(0x{CANCTR_VOLTAGE_STATUS_ADDR:03X}) on bus {voltage_bus}, and CANCTR_IOStatus "
      + f"(0x{CANCTR_IO_STATUS_ADDR:03X}) on bus {voltage_bus}. Press Ctrl-C to stop."
    )
  elif not display.interactive:
    display.update(display.latest_message)
  while True:
    for addr, _, payload, rx_bus in panda.can_recv():
      if addr == VSM2_ADDR and rx_bus == torque_bus:
        steering_torque, output_torque = decode_vsm2_torques(payload)
      elif addr == CANCTR_VOLTAGE_STATUS_ADDR and rx_bus == voltage_bus:
        snr1_voltage, snr2_voltage, eps1_voltage, eps2_voltage = decode_voltage_status(payload)
      elif addr == CANCTR_IO_STATUS_ADDR and rx_bus == voltage_bus:
        io_status = decode_io_status(payload)
        ref_voltage = io_status["eps_refv"]
        if status_cache is not None:
          status_cache.update_io_status(io_status)
      else:
        continue

      elapsed = time.monotonic() - start
      message = build_monitor_message(
        torque_bus, voltage_bus, elapsed, steering_torque, output_torque,
        snr1_voltage, snr2_voltage, eps1_voltage, eps2_voltage, ref_voltage,
      )
      if display is None:
        print(message)
      else:
        display.update(message)


def start_monitor_thread(panda: Panda, torque_bus: int, voltage_bus: int,
                         display: MonitorDisplay | None = None,
                         status_cache: CanctrStatusCache | None = None) -> threading.Thread:
  thread = threading.Thread(target=monitor_vsm2, args=(panda, torque_bus, voltage_bus, display, status_cache), daemon=True)
  thread.start()
  return thread


def prompt_value(label: str, parser, default):
  while True:
    default_text = f" [{default}]" if default is not None else ""
    raw = input(f"{label}{default_text}: ").strip()
    if raw.lower() in EXIT_WORDS:
      raise KeyboardInterrupt
    if raw == "" and default is not None:
      return default
    try:
      return parser(raw)
    except ValueError as err:
      print(f"Invalid input: {err}")


def print_state(rel: int, rele: int, command_value: float, torque_mode: bool) -> None:
  requested_voltage = command_to_voltage(command_value, torque_mode)
  delta, saturated = voltage_to_delta(requested_voltage)
  applied_voltage = delta_to_voltage(delta)
  applied_command_value = voltage_to_command(applied_voltage, torque_mode)
  clamp_text = " [clamped]" if saturated else ""
  extra_text = ""
  if torque_mode:
    extra_text = f" legacy_input={voltage_to_input(applied_voltage):+.6f}"
  print(
    f"Current state: REL={rel} RELE={rele} {mode_label(torque_mode)}={command_value:+.6f} "
    + f"-> voltage={applied_voltage:+.6f} V -> delta={delta:+d} "
    + f"(equivalent {mode_label(torque_mode)} {applied_command_value:+.6f}){clamp_text}{extra_text}"
  )


def print_interactive_help(max_command_value: float, torque_mode: bool, period_ms: float, status_bus: int) -> None:
  print("Interactive commands:")
  print(f"  <value>          send a new {mode_label(torque_mode)} immediately, limited to +/-{max_command_value:.6f}")
  if torque_mode:
    print(
      f"                   torque mode mapping: {TORQUE_REFERENCE:.0f} torque = "
      + f"{LEGACY_INPUT_AT_TORQUE_REFERENCE:.0f} legacy input = {LEGACY_INPUT_AT_TORQUE_REFERENCE / INPUT_SCALE:.3f} V"
    )
  print(f"  <Enter>          resend the current {mode_label(torque_mode)} immediately")
  print("  rel <0|1>        update REL and send immediately")
  print("  rele <0|1>       update RELE and send immediately")
  print("  flags <r> <e>    update both relay bits and send immediately")
  print("  show             print current state")
  print(f"  err/errors       show latest decoded CANCTR_IOStatus from bus {status_bus}")
  print("  help             show this help")
  print("  q                quit")
  print(f"Keepalive: current command is resent every {period_ms:.0f} ms while the script runs.")


def run_interactive(panda: Panda, bus: int, command_state: CommandState,
                    send_period_s: float,
                    prompt_for_rel: bool, prompt_for_rele: bool,
                    monitor_display: MonitorDisplay | None = None,
                    status_cache: CanctrStatusCache | None = None,
                    status_bus: int = VOLTAGE_MONITOR_BUS) -> None:
  rel, rele, command_value, torque_mode = command_state.snapshot()
  max_delta_voltage = delta_to_voltage(MAX_DELTA)
  max_command_value = abs(voltage_to_command(max_delta_voltage, torque_mode))
  if prompt_for_rel:
    rel = prompt_value("REL bit", parse_bit, rel)
  if prompt_for_rele:
    rele = prompt_value("RELE bit", parse_bit, rele)
  rel, rele, command_value, torque_mode = command_state.update(rel=rel, rele=rele)

  print("Enter q to quit.")
  if torque_mode:
    print(
      f"Torque is first mapped as {TORQUE_REFERENCE:.0f} torque = {LEGACY_INPUT_AT_TORQUE_REFERENCE:.0f} legacy input, "
      + f"then applied as voltage = -input/{INPUT_SCALE:.0f}; limited to +/-{max_command_value:.6f} torque units."
    )
    print(
      f"This means {TORQUE_REFERENCE:.0f} torque -> {LEGACY_INPUT_AT_TORQUE_REFERENCE:.0f} input -> "
      + f"{LEGACY_INPUT_AT_TORQUE_REFERENCE / INPUT_SCALE:.3f} V."
    )
  else:
    print(
      f"Input is scaled and sign-flipped before transmit: applied voltage = -input/{INPUT_SCALE:.0f}, "
      + f"limited to +/-{max_command_value:.6f} input units."
    )
  print_interactive_help(max_command_value, torque_mode, send_period_s * 1000.0, status_bus)
  print_state(rel, rele, command_value, torque_mode)
  print(f"Initial command is sent immediately, then kept alive every {send_period_s * 1000.0:.0f} ms.")
  print()

  send_command(panda, bus, *command_state.next_frame_params())
  stop_event = threading.Event()
  keepalive_thread = start_keepalive_thread(panda, bus, command_state, send_period_s, stop_event)

  try:
    while True:
      if monitor_display is not None:
        monitor_display.show_prompt_header()
        monitor_display.set_prompt_active(True)

      try:
        raw = input(mode_prompt(torque_mode)).strip()
      finally:
        if monitor_display is not None:
          monitor_display.set_prompt_active(False)

      normalized = raw.lower()
      if normalized in EXIT_WORDS:
        raise KeyboardInterrupt

      try:
        if raw == "":
          send_command(panda, bus, *command_state.next_frame_params())
        elif normalized in {"h", "help", "?"}:
          print_interactive_help(max_command_value, torque_mode, send_period_s * 1000.0, status_bus)
        elif normalized in {"s", "show", "status"}:
          print_state(*command_state.snapshot())
        elif normalized in {"err", "errs", "error", "errors", "fault", "faults"}:
          print_canctr_io_status(panda, status_bus, status_cache)
        else:
          tokens = raw.split()
          command = tokens[0].lower()
          if command == "rel" and len(tokens) == 2:
            command_state.update(rel=parse_bit(tokens[1]))
            send_command(panda, bus, *command_state.next_frame_params())
          elif command == "rele" and len(tokens) == 2:
            command_state.update(rele=parse_bit(tokens[1]))
            send_command(panda, bus, *command_state.next_frame_params())
          elif command in {"flags", "bits"} and len(tokens) == 3:
            command_state.update(rel=parse_bit(tokens[1]), rele=parse_bit(tokens[2]))
            send_command(panda, bus, *command_state.next_frame_params())
          elif command in {"send", "input", "voltage", "v", "torque", "tq"} and len(tokens) == 2:
            command_state.update(command_value=float(tokens[1]))
            send_command(panda, bus, *command_state.next_frame_params())
          elif len(tokens) == 1:
            command_state.update(command_value=float(tokens[0]))
            send_command(panda, bus, *command_state.next_frame_params())
          else:
            raise ValueError(f"enter a {mode_label(torque_mode)} value, or use rel/rele/flags/show/err/help")
      except ValueError as err:
        print(f"Invalid input: {err}")

      print()
  finally:
    stop_event.set()
    keepalive_thread.join(timeout=1.0)


def main() -> None:
  parser = argparse.ArgumentParser(
    description="Send guarded CANCTR delta-control frames (0x231) directly with Panda, including the byte-6 checksum and a zeroed openpilot-limit byte. By default the script keeps resending the last command every 100 ms so the STM32 CANCTR watchdog stays fed. Optional --TQ mode interprets values as torque using 100 torque = 165 legacy input = 0.165 V.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  )
  parser.add_argument("--bus", type=int, default=DEFAULT_BUS, help="CAN bus to send the command on")
  parser.add_argument("--rel", type=parse_bit, help="REL bit: 0/1, true/false, yes/no")
  parser.add_argument("--rele", type=parse_bit, help="RELE bit: 0/1, true/false, yes/no")
  parser.add_argument(
    "--voltage",
    type=float,
    help=f"Command value to send. Default mode uses applied voltage = -input/{INPUT_SCALE:.0f}; with --TQ, {TORQUE_REFERENCE:.0f} torque = {LEGACY_INPUT_AT_TORQUE_REFERENCE:.0f} legacy input = {LEGACY_INPUT_AT_TORQUE_REFERENCE / INPUT_SCALE:.3f} V",
  )
  parser.add_argument(
    "--TQ", "--tq",
    dest="torque_mode",
    action="store_true",
    help=f"Interpret --voltage and interactive values as torque; {TORQUE_REFERENCE:.0f} torque = {LEGACY_INPUT_AT_TORQUE_REFERENCE:.0f} legacy input = {LEGACY_INPUT_AT_TORQUE_REFERENCE / INPUT_SCALE:.3f} V",
  )
  parser.add_argument(
    "--interactive",
    action="store_true",
    help="Prompt for REL/RELE/input values in a loop",
  )
  parser.add_argument(
    "--no-interactive",
    action="store_true",
    help="Disable interactive mode",
  )
  parser.add_argument(
    "--period-ms",
    type=float,
    default=DEFAULT_KEEPALIVE_PERIOD_MS,
    help=f"Keepalive resend period in milliseconds; must stay below the CANCTR watchdog timeout of {CANCTR_WATCHDOG_TIMEOUT_MS:.0f} ms",
  )
  parser.add_argument(
    "--once",
    action="store_true",
    help="Send one command frame and exit instead of periodically resending it",
  )
  parser.add_argument(
    "--monitor",
    action="store_true",
    help="Track VSM2 torques on bus 0 plus EPS1/EPS2/SNR1/SNR2 from 0x243 and EPS_REFV from 0x241 on bus 1",
  )
  parser.add_argument(
    "--no-monitor",
    action="store_true",
    help="Disable VSM2 monitoring",
  )
  args = parser.parse_args()

  interactive = not args.no_interactive
  if args.interactive:
    interactive = True

  if args.period_ms <= 0.0:
    parser.error("--period-ms must be greater than 0")
  if (not args.once) and (args.period_ms >= CANCTR_WATCHDOG_TIMEOUT_MS):
    parser.error(
      f"--period-ms must stay below the CANCTR watchdog timeout of {CANCTR_WATCHDOG_TIMEOUT_MS:.0f} ms unless --once is used"
    )
  if interactive and args.once:
    parser.error("--once cannot be used with interactive mode")

  send_period_s = args.period_ms / 1000.0

  panda = Panda()
  enable_output(panda)

  rel = 0 if args.rel is None else args.rel
  rele = 0 if args.rele is None else args.rele
  command_value = 0.0 if args.voltage is None else args.voltage
  torque_mode = args.torque_mode
  command_state = CommandState(rel, rele, command_value, torque_mode)

  monitor_enabled = not args.no_monitor
  if args.monitor:
    monitor_enabled = True

  monitor_display = MonitorDisplay(TORQUE_MONITOR_BUS, VOLTAGE_MONITOR_BUS, interactive=interactive) if monitor_enabled else None
  status_cache = CanctrStatusCache() if monitor_enabled else None

  keepalive_stop = threading.Event()
  keepalive_thread = None

  try:
    if interactive:
      if monitor_enabled:
        print("VSM2 monitor enabled. Torque values refresh on a single status line above the prompt.")
        start_monitor_thread(panda, TORQUE_MONITOR_BUS, VOLTAGE_MONITOR_BUS, monitor_display, status_cache)
      run_interactive(
        panda,
        args.bus,
        command_state,
        send_period_s,
        prompt_for_rel=args.rel is None,
        prompt_for_rele=args.rele is None,
        monitor_display=monitor_display,
        status_cache=status_cache,
        status_bus=VOLTAGE_MONITOR_BUS if monitor_enabled else args.bus,
      )
    else:
      send_command(panda, args.bus, *command_state.next_frame_params())
      if not args.once:
        print(f"Keepalive active: resending the current command every {args.period_ms:.0f} ms. Press Ctrl-C to stop.")
        keepalive_thread = start_keepalive_thread(panda, args.bus, command_state, send_period_s, keepalive_stop)
      if monitor_enabled:
        monitor_vsm2(panda, TORQUE_MONITOR_BUS, VOLTAGE_MONITOR_BUS, monitor_display)
      elif not args.once:
        while True:
          time.sleep(1.0)
  except KeyboardInterrupt:
    print("\nExiting.")
  finally:
    keepalive_stop.set()
    if keepalive_thread is not None:
      keepalive_thread.join(timeout=1.0)
    if monitor_display is not None:
      monitor_display.finish()
    panda.set_safety_mode(Panda.SAFETY_SILENT)


if __name__ == "__main__":
  main()
