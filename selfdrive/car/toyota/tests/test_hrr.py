from types import SimpleNamespace

from cereal import car
from panda import Panda

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.can.tests.test_packer_parser import can_list_to_can_capnp
from openpilot.selfdrive.car.toyota import toyotacan
from openpilot.selfdrive.car.toyota.carcontroller import CarController
from openpilot.selfdrive.car.toyota.carstate import (LS600H_HRR_STATUS_MAX_AGE_FRAMES, LS600H_HRR_TEMPERATURE_MAX_AGE_FRAMES,
                                                      LS600H_HRR_WRAP_HOLD_MAX_FRAMES, ls600h_hrr_ecu_overtemperature,
                                                      ls600h_hrr_steering_valid, ls600h_hrr_wrap_holdover_valid)
from openpilot.selfdrive.car.toyota.values import CAR, DBC, ToyotaFlags


def build_controller():
  CP = car.CarParams.new_message()
  CP.carFingerprint = CAR.LEXUS_LS600h
  CP.carName = "toyota"
  CP.flags = int(ToyotaFlags.UNSUPPORTED_DSU | ToyotaFlags.RAISED_ACCEL_LIMIT)
  CP.lateralTuning.init("torque")
  CP.openpilotLongitudinalControl = False
  return CarController(DBC[CAR.LEXUS_LS600h]["pt"], CP, None)


def build_car_state():
  out = car.CarState.new_message()
  out.gearShifter = car.CarState.GearShifter.park
  return SimpleNamespace(
    out=out,
    pcm_acc_status=0,
    pcm_follow_distance=0,
    acc_type=1,
    gvc=0.0,
    lkas_hud={},
  )


def build_toggles():
  return SimpleNamespace(sng_hack=False, lock_doors=False, unlock_doors=False)


def test_ls600h_params_enable_hrr_steering_with_stock_longitudinal():
  from openpilot.selfdrive.car.toyota.interface import CarInterface

  toggles = SimpleNamespace(disable_openpilot_long=False)
  CP = CarInterface.get_params(CAR.LEXUS_LS600h, {0: {}, 1: {}, 2: {}}, [], False, toggles)

  assert CP.safetyConfigs[0].safetyModel == car.CarParams.SafetyModel.toyota
  assert CP.safetyConfigs[0].safetyParam & Panda.FLAG_TOYOTA_HRR
  assert CP.safetyConfigs[0].safetyParam & Panda.FLAG_TOYOTA_STOCK_LONGITUDINAL
  assert not CP.safetyConfigs[0].safetyParam & Panda.FLAG_TOYOTA_ALT_BRAKE
  assert CP.pcmCruise
  assert not CP.openpilotLongitudinalControl


def test_ls600h_hrr_true_angle_status_requirements():
  status = {
    "True_Angle_Valid": 1,
    "True_Angle_Initialized": 1,
    "True_Angle_Resolver_Valid": 1,
    "True_Angle_Calibrated": 1,
    "True_Angle_Wrap_Ambiguous": 0,
  }
  assert ls600h_hrr_steering_valid(status, LS600H_HRR_STATUS_MAX_AGE_FRAMES)

  for signal in ("True_Angle_Valid", "True_Angle_Initialized",
                 "True_Angle_Resolver_Valid", "True_Angle_Calibrated"):
    invalid_status = status | {signal: 0}
    assert not ls600h_hrr_steering_valid(invalid_status, 0)

  assert not ls600h_hrr_steering_valid(status | {"True_Angle_Wrap_Ambiguous": 1}, 0)
  assert not ls600h_hrr_steering_valid(status, LS600H_HRR_STATUS_MAX_AGE_FRAMES + 1)


def test_ls600h_hrr_true_angle_dbc_layout():
  parser = CANParser(DBC[CAR.LEXUS_LS600h]["pt"], [
    ("HRR_TrueAngleStatus", 0),
    ("HRR_PerformanceStatus", 0),
  ], 0)
  packer = CANPacker(DBC[CAR.LEXUS_LS600h]["pt"])
  msg = packer.make_can_msg("HRR_TrueAngleStatus", 0, {
    "True_Steering_Angle": -3.08,
    "DLY_Samples": 42,
  })
  parser.update_strings([can_list_to_can_capnp([msg])])

  assert parser.vl["HRR_TrueAngleStatus"]["True_Steering_Angle"] == -3.08
  assert parser.vl["HRR_TrueAngleStatus"]["DLY_Samples"] == 42


def test_ls600h_hrr_wrap_holdover_requirements():
  wrap_ambiguous = {
    "True_Angle_Resolver_Valid": 1,
    "True_Angle_Calibrated": 1,
    "True_Angle_Wrap_Ambiguous": 1,
  }
  assert ls600h_hrr_wrap_holdover_valid(wrap_ambiguous, 0, LS600H_HRR_WRAP_HOLD_MAX_FRAMES)
  assert not ls600h_hrr_wrap_holdover_valid(wrap_ambiguous, 1, 1)
  assert not ls600h_hrr_wrap_holdover_valid(wrap_ambiguous, 0, LS600H_HRR_WRAP_HOLD_MAX_FRAMES + 1)
  assert not ls600h_hrr_wrap_holdover_valid(wrap_ambiguous | {"True_Angle_Resolver_Valid": 0}, 0, 1)
  assert not ls600h_hrr_wrap_holdover_valid(wrap_ambiguous | {"True_Angle_Calibrated": 0}, 0, 1)
  assert not ls600h_hrr_wrap_holdover_valid(wrap_ambiguous | {"True_Angle_Wrap_Ambiguous": 0}, 0, 1)


def test_ls600h_hrr_ecu_high_temperature_warning_requirements():
  status = {"MCU_Temperature": 86, "MCU_Temperature_Valid": 1}
  assert ls600h_hrr_ecu_overtemperature(status, LS600H_HRR_TEMPERATURE_MAX_AGE_FRAMES)
  assert not ls600h_hrr_ecu_overtemperature(status | {"MCU_Temperature": 85}, 0)
  assert not ls600h_hrr_ecu_overtemperature(status | {"MCU_Temperature_Valid": 0}, 0)
  assert not ls600h_hrr_ecu_overtemperature(status, LS600H_HRR_TEMPERATURE_MAX_AGE_FRAMES + 1)


def test_ls600h_controller_sends_hrr_torque():
  controller = build_controller()
  CS = build_car_state()
  toggles = build_toggles()

  inactive = car.CarControl.new_message()
  _, sends = controller.update(inactive.as_reader(), CS, 0, toggles)
  assert len(sends) == 1
  assert sends[0][0] == 0x160
  assert sends[0][3] == 2
  assert sends[0][2][4] & 0x3 == 0

  active = car.CarControl.new_message()
  active.enabled = True
  active.latActive = True
  active.actuators.steer = 0.5
  actuators, sends = controller.update(active.as_reader(), CS, 0, toggles)

  assert len(sends) == 1
  address, _, payload, bus = sends[0]
  assert address == 0x160
  assert bus == 2
  assert len(payload) == 7
  assert payload[4] & 0x3 == 0x3

  torque_raw = payload[0] | ((payload[1] & 0xF) << 8)
  assert torque_raw == 6  # first active command is limited by TOYOTA_HRR_DELTA_UP
  assert actuators.steerOutputCan == 6


def test_ls600h_hrr_torque_command_clamps_to_scaled_range():
  assert toyotacan.clamp_hrr_torque_demand(801) == 800
  assert toyotacan.clamp_hrr_torque_demand(-801) == -800
