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
