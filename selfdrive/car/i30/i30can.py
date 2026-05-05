import struct

from openpilot.selfdrive.car import make_can_msg
from openpilot.selfdrive.car.i30.values import TrqiSteerLimitParams

TRQI_TORQUE_ADDR = 0x232
TRQI_CAN_BUS = 1
TRQI_LIMIT_FLAG_STEER_DELTA_UP = 0x01
TRQI_LIMIT_FLAG_STEER_DELTA_DOWN = 0x02
TRQI_LIMIT_FLAG_STEER_MAX = 0x04
TRQI_LIMIT_FLAG_OUT_TQ_LIMITED = 0x08


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


def clamp_trqi_torque_demand(command_tq: int) -> int:
  return max(-2048, min(2047, command_tq))


def compute_trqi_crc8(addr: int, payload_without_checksum: bytes) -> int:
  crc = 0x00
  for byte in ((addr & 0xFF), ((addr >> 8) & 0xFF), *payload_without_checksum):
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ 0x07) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc


def create_trqi_torque_command(command_tq: float, lat_active: bool, counter: int, op_limit_flags: int = 0):
  # 0x232 carries desired EPS output torque as signed Ncm in the low 12 bits
  # of bytes0..1, the raw12 ones-complement in bytes2..3, and a CRC-8 in byte6.
  torque_ncm = clamp_trqi_torque_demand(int(round(command_tq)))
  torque_raw = torque_ncm & 0x0FFF
  torque_complement_raw = torque_raw ^ 0x0FFF
  rel = TrqiSteerLimitParams.RELAY_ENABLED if lat_active else 0
  rele = TrqiSteerLimitParams.RELAYE_ENABLED if lat_active else 0
  flags = (rel & 0x1) | ((rele & 0x1) << 1) | ((op_limit_flags & 0x0F) << 2)
  counter_byte = counter & 0x0F
  payload_without_checksum = struct.pack("<HHBB", torque_raw, torque_complement_raw, flags, counter_byte)
  checksum = compute_trqi_crc8(TRQI_TORQUE_ADDR, payload_without_checksum)
  # Use the standard Openpilot CAN tuple shape: [address, busTime, dat, src].
  # card.py/sendcan -> can_list_to_can_capnp expects all four fields.
  return make_can_msg(TRQI_TORQUE_ADDR, payload_without_checksum + bytes([checksum]), TRQI_CAN_BUS)
