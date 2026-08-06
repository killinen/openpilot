import struct

from cereal import car
from openpilot.selfdrive.car import make_can_msg

SteerControlType = car.CarParams.SteerControlType

HRR_TORQUE_ADDR = 0x160
HRR_CAN_BUS = 2
HRR_LIMIT_FLAG_STEER_DELTA_UP = 0x01
HRR_LIMIT_FLAG_STEER_DELTA_DOWN = 0x02
HRR_LIMIT_FLAG_STEER_MAX = 0x04
HRR_LIMIT_FLAG_OUT_TQ_LIMITED = 0x08
HRR_RELAY_ENABLED = 1
HRR_RELAYE_ENABLED = 1
HRR_MAX_TORQUE_NCM = 800  # 8 Nm


def create_steer_command(packer, steer, steer_req):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)


def create_lta_steer_command(packer, steer_control_type, steer_angle, steer_req, frame, torque_wind_down):
  """Creates a CAN message for the Toyota LTA Steer Command."""

  values = {
    "COUNTER": frame + 128,
    "SETME_X1": 1,  # suspected LTA feature availability
    # 1 for TSS 2.5 cars, 3 for TSS 2.0. Send based on whether we're using LTA for lateral control
    "SETME_X3": 1 if steer_control_type == SteerControlType.angle else 3,
    "PERCENTAGE": 100,
    "TORQUE_WIND_DOWN": torque_wind_down,
    "ANGLE": 0,
    "STEER_ANGLE_CMD": steer_angle,
    "STEER_REQUEST": steer_req,
    "STEER_REQUEST_2": steer_req,
    "CLEAR_HOLD_STEERING_ALERT": 0,
  }
  return packer.make_can_msg("STEERING_LTA", 0, values)


def create_lta_steer_command_2(packer, frame):
  values = {
    "COUNTER": frame + 128,
  }
  return packer.make_can_msg("STEERING_LTA_2", 0, values)


def clamp_hrr_torque_demand(command_tq: int) -> int:
  return max(-HRR_MAX_TORQUE_NCM, min(HRR_MAX_TORQUE_NCM, command_tq))


def compute_hrr_crc8(addr: int, payload_without_checksum: bytes) -> int:
  crc = 0x00
  for byte in ((addr & 0xFF), ((addr >> 8) & 0xFF), *payload_without_checksum):
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ 0x07) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc


def create_hrr_torque_command(command_tq: float, lat_active: bool, counter: int, op_limit_flags: int = 0):
  torque_ncm = clamp_hrr_torque_demand(int(round(command_tq)))
  torque_raw = torque_ncm & 0x0FFF
  torque_complement_raw = torque_raw ^ 0x0FFF
  rel = HRR_RELAY_ENABLED if lat_active else 0
  rele = HRR_RELAYE_ENABLED if lat_active else 0
  flags = (rel & 0x1) | ((rele & 0x1) << 1) | ((op_limit_flags & 0x0F) << 2)
  counter_byte = counter & 0x0F
  payload_without_checksum = struct.pack("<HHBB", torque_raw, torque_complement_raw, flags, counter_byte)
  checksum = compute_hrr_crc8(HRR_TORQUE_ADDR, payload_without_checksum)
  return make_can_msg(HRR_TORQUE_ADDR, payload_without_checksum + bytes([checksum]), HRR_CAN_BUS)


def create_accel_command(packer, accel, pcm_cancel, permit_braking, standstill_req, lead, acc_type, fcw_alert, distance, reverse_cruise_active):
  # TODO: find the exact canceling bit that does not create a chime
  values = {
    "ACCEL_CMD": accel,
    "ACC_TYPE": acc_type,
    "DISTANCE": distance,
    "MINI_CAR": lead,
    "PERMIT_BRAKING": permit_braking,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
    "ALLOW_LONG_PRESS": 2 if reverse_cruise_active else 1,
    "ACC_CUT_IN": fcw_alert,  # only shown when ACC enabled
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)

def create_accel_command_2(packer, accel):
  values = {
    "ACCEL_CMD": accel,
  }
  return packer.make_can_msg("ACC_CONTROL_2", 0, values)

def create_pcs_commands(packer, accel, active, mass):
  values1 = {
    "COUNTER": 0,
    "FORCE": round(min(accel, 0) * mass * 2),
    "STATE": 3 if active else 0,
    "BRAKE_STATUS": 0,
    "PRECOLLISION_ACTIVE": 1 if active else 0,
  }
  msg1 = packer.make_can_msg("PRE_COLLISION", 0, values1)

  values2 = {
    "DSS1GDRV": min(accel, 0),     # accel
    "PCSALM": 1 if active else 0,  # goes high same time as PRECOLLISION_ACTIVE
    "IBTRGR": 1 if active else 0,  # unknown
    "PBATRGR": 1 if active else 0, # noisy actuation bit?
    "PREFILL": 1 if active else 0, # goes on and off before DSS1GDRV
    "AVSTRGR": 1 if active else 0,
  }
  msg2 = packer.make_can_msg("PRE_COLLISION_2", 0, values2)

  return [msg1, msg2]


def create_acc_cancel_command(packer):
  values = {
    "GAS_RELEASED": 0,
    "CRUISE_ACTIVE": 0,
    "ACC_BRAKING": 0,
    "ACCEL_NET": 0,
    "CRUISE_STATE": 0,
    "CANCEL_REQ": 1,
  }
  return packer.make_can_msg("PCM_CRUISE", 0, values)


def create_fcw_command(packer, fcw):
  values = {
    "PCS_INDICATOR": 1,  # PCS turned off
    "FCW": fcw,
    "SET_ME_X20": 0x20,
    "SET_ME_X10": 0x10,
    "PCS_OFF": 1,
    "PCS_SENSITIVITY": 0,
  }
  return packer.make_can_msg("PCS_HUD", 0, values)


def create_ui_command(packer, steer, chime, left_line, right_line, left_lane_depart, right_lane_depart, enabled, stock_lkas_hud, lat_active):
  values = {
    "TWO_BEEPS": chime,
    "LDA_ALERT": steer,
    "RIGHT_LINE": 0 if not lat_active else 3 if right_lane_depart else 1 if right_line else 2,
    "LEFT_LINE": 0 if not lat_active else 3 if left_lane_depart else 1 if left_line else 2,
    "BARRIERS": 1 if lat_active else 0,

    # static signals
    "SET_ME_X02": 2,
    "SET_ME_X01": 1,
    "LKAS_STATUS": 1,
    "REPEATED_BEEPS": 0,
    "LANE_SWAY_FLD": 7,
    "LANE_SWAY_BUZZER": 0,
    "LANE_SWAY_WARNING": 0,
    "LDA_FRONT_CAMERA_BLOCKED": 0,
    "TAKE_CONTROL": 0,
    "LANE_SWAY_SENSITIVITY": 2,
    "LANE_SWAY_TOGGLE": 1,
    "LDA_ON_MESSAGE": 0,
    "LDA_MESSAGES": 0,
    "LDA_SA_TOGGLE": 1,
    "LDA_SENSITIVITY": 2,
    "LDA_UNAVAILABLE": 0,
    "LDA_MALFUNCTION": 0,
    "LDA_UNAVAILABLE_QUIET": 0,
    "ADJUSTING_CAMERA": 0,
    "LDW_EXIST": 1,
  }

  # lane sway functionality
  # not all cars have LKAS_HUD — update with camera values if available
  if len(stock_lkas_hud):
    values.update({s: stock_lkas_hud[s] for s in [
      "LANE_SWAY_FLD",
      "LANE_SWAY_BUZZER",
      "LANE_SWAY_WARNING",
      "LANE_SWAY_SENSITIVITY",
      "LANE_SWAY_TOGGLE",
    ]})

  return packer.make_can_msg("LKAS_HUD", 0, values)
