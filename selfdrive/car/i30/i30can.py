import struct

from openpilot.selfdrive.car import make_can_msg
from openpilot.selfdrive.car.i30.values import TrqiSteerLimitParams

TRQI_DELTA_ADDR = 0x231
TRQI_CAN_BUS = 1
TRQI_DAC_MAX_CODE = (1 << TrqiSteerLimitParams.DAC_BITS) - 1
TRQI_LIMIT_FLAG_STEER_DELTA_UP = 0x01
TRQI_LIMIT_FLAG_STEER_DELTA_DOWN = 0x02
TRQI_LIMIT_FLAG_STEER_MAX = 0x04


# Simple checksum helper copied from the Hyundai implementation. The i30 actuator
# expects the same checksum scheme on STEERING_COMMAND.
def calc_checksum_8bit(data: bytes, msg_id: int) -> int:
  checksum = msg_id
  for byte in data:
    checksum += byte

  checksum = (checksum & 0xFF) + (checksum >> 8)
  return checksum & 0xFF


def crc8_pedal(data):
  crc = 0xFF
  poly = 0xD5
  size = len(data)
  for i in range(size - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if (crc & 0x80) != 0:
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc


def create_gas_interceptor_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 100.
    values["GAS_COMMAND2"] = gas_amount * 100.

  dat = packer.make_can_msg("GAS_COMMAND", 1, values)[2]

  checksum = crc8_pedal(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 1, values)


def create_steer_command(packer, mode: int, steer_delta: float, steer_tq: float, frame: int):
  """Create SSC steering command for the i30 steering actuator."""
  values = {
    "SERVO_COUNTER": frame % 0xF,
    "STEER_MODE": mode,
    "STEER_ANGLE": steer_delta,
    "STEER_TORQUE": steer_tq,
  }
  msg = packer.make_can_msg("STEERING_COMMAND", 0, values)
  addr = msg[0]
  dat = msg[2]

  values["SERVO_CHECKSUM"] = calc_checksum_8bit(dat, addr)

  # bus 1 is the actuator CAN bus
  return packer.make_can_msg("STEERING_COMMAND", 1, values)


def clamp_trqi_delta(delta: int) -> int:
  return max(-TrqiSteerLimitParams.MAX_DELTA, min(TrqiSteerLimitParams.MAX_DELTA, delta))


def compute_trqi_checksum(addr: int, payload_without_checksum: bytes) -> int:
  # TRQI delta frames use the board's simple low8(id_lo + id_hi + bytes0..5) checksum.
  total = (addr & 0xFF) + ((addr >> 8) & 0xFF) + sum(payload_without_checksum)
  return total & 0xFF


def trqi_torque_to_delta(command_tq: float) -> int:
  # Match send_canctr_delta.py exactly:
  #   TQ -> legacy input -> voltage -> raw DAC delta counts.
  legacy_input = command_tq * TrqiSteerLimitParams.LEGACY_INPUT_AT_TORQUE_REFERENCE / TrqiSteerLimitParams.TORQUE_REFERENCE
  voltage = -legacy_input / TrqiSteerLimitParams.INPUT_SCALE
  delta = int(round(voltage * TRQI_DAC_MAX_CODE / TrqiSteerLimitParams.DAC_FULL_SCALE_VOLTS))
  return clamp_trqi_delta(delta)


def create_trqi_steer_command(command_tq: float, lat_active: bool, counter: int, op_limit_flags: int = 0):
  # Positive command_tq is defined as a right-turn request for TRQI mode.
  # The sender-compatible conversion above intentionally makes that become a
  # negative voltage / delta on the wire, which matches the standalone tool.
  delta = trqi_torque_to_delta(command_tq)
  rel = TrqiSteerLimitParams.RELAY_ENABLED if lat_active else 0
  rele = TrqiSteerLimitParams.RELAYE_ENABLED if lat_active else 0
  # Byte 4 carries relay control plus markpilot-side limit information so
  # captured 0x231 traffic shows whether openpilot clipped the outgoing TRQI
  # request before it reached the actuator. Byte 5 carries only the rolling
  # counter in its lower nibble so the board and Panda can sequence-check it.
  flags = (rel & 0x1) | ((rele & 0x1) << 1) | ((op_limit_flags & 0x07) << 2)
  counter_byte = counter & 0x0F
  payload_without_checksum = struct.pack("<hhBB", delta, delta, flags, counter_byte)
  checksum = compute_trqi_checksum(TRQI_DELTA_ADDR, payload_without_checksum)
  # Use the standard Openpilot CAN tuple shape: [address, busTime, dat, src].
  # card.py/sendcan -> can_list_to_can_capnp expects all four fields.
  return make_can_msg(TRQI_DELTA_ADDR, payload_without_checksum + bytes([checksum]), TRQI_CAN_BUS)
