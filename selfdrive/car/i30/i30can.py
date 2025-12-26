# Simple checksum helper copied from the Hyundai implementation. The i30 actuator
# expects the same checksum scheme on STEERING_COMMAND.
def calc_checksum_8bit(data: bytes, msg_id: int) -> int:
  checksum = msg_id
  for byte in data:
    checksum += byte

  checksum = (checksum & 0xFF) + (checksum >> 8)
  return checksum & 0xFF


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
