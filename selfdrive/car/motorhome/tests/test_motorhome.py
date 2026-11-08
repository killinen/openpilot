"""Native DBC/interface/controller checks, run in the disposable Docker checkout."""

from types import SimpleNamespace

import pytest

from cereal import car, custom, log
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from openpilot.frogpilot.common.frogpilot_variables import _DEFAULT_TOGGLE_VALUES
from openpilot.frogpilot.controls.frogpilot_card import FrogPilotCard
from openpilot.selfdrive.car import gen_empty_fingerprint
from openpilot.selfdrive.car.i30.i30can import calc_checksum_8bit
from openpilot.selfdrive.car.motorhome.carcontroller import CarController
from openpilot.selfdrive.car.motorhome.carstate import CarState
from openpilot.selfdrive.car.motorhome.interface import CarInterface
from openpilot.selfdrive.car.motorhome.values import CAR
from panda import ALTERNATIVE_EXPERIENCE
from panda import Panda
from panda.tests.libpanda import libpanda_py


def can_event(messages, nanos):
  event = log.Event.new_message(logMonoTime=nanos, valid=True)
  event.init("can", len(messages))
  for frame, (address, _, data, bus) in zip(event.can, messages, strict=True):
    frame.address, frame.src, frame.dat = address, bus, data
  return event.to_bytes()


class Harness:
  def __init__(self, ssc=False):
    self.toggles = SimpleNamespace(**{**_DEFAULT_TOGGLE_VALUES, "motorhome_ssc_enabled": ssc,
      "always_on_lateral": True, "always_on_lateral_set": True, "always_on_lateral_main": True})
    self.CP = CarInterface.get_params(CAR.MOTORHOME_J1939_SSC, gen_empty_fingerprint(), [], False, self.toggles)
    self.CP.alternativeExperience = ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL | ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS
    self.CI = CarInterface(self.CP, custom.FrogPilotCarParams.new_message(), CarController, CarState)
    self.packer = CANPacker("j1939_standard")
    self.ssc_packer = CANPacker("hyundai_i30_2014")
    self.nanos = 1_000_000_000
    self.counter = 0
    self.aol = FrogPilotCard(SimpleNamespace(CP=self.CP, frogpilot_toggles=self.toggles))
    self.sm = type("Messages", (dict,), {"updated": {"frogpilotPlan": False}})(
      frogpilotPlan=SimpleNamespace(lateralCheck=True), liveCalibration=SimpleNamespace(calPerc=100),
      onroadEvents=[], frogpilotOnroadEvents=[])

  def ssc_message(self, status=2, corrupt=False):
    values = {"SERVO_COUNTER": self.counter, "CONTROL_STATUS": status, "STEERING_ANGLE": 12.5,
              "STEERING_SPEED": .25, "STEERING_TORQUE": .5, "TEMPERATURE": 20}
    data = self.ssc_packer.make_can_msg("STEERING_STATUS", 1, values)[2]
    values["SERVO_CHECKSUM"] = calc_checksum_8bit(data[1:7], 0x22F) ^ int(corrupt)
    self.counter = (self.counter + 1) % 16
    return self.ssc_packer.make_can_msg("STEERING_STATUS", 1, values)

  def step(self, main=1, cruise=0, brake=0, gas=0, switches=0, speed=50, set_speed=49,
           ssc_status=2, ssc_present=True, corrupt_ssc=False, omit=(), gear=6, current_gear=None):
    master = {"CRUISE_CONTROL_ENABLE_SWITCH": main, "BRAKE_SWITCH": brake}
    for index, name in enumerate(("SET", "COAST", "RESUME", "ACCELERATE")):
      master[f"CRUISE_CONTROL_{name}_SWITCH"] = (switches >> (index * 2)) & 3
    values = [("CCVS_00", {"CRUISE_CONTROL_ACTIVE": cruise, "WHEEL_BASED_VEHICLE_SPEED": speed,
                            "CRUISE_CONTROL_SET_SPEED": set_speed}), ("CCVS_17", master),
              ("CCVS_21", {"BRAKE_SWITCH": brake}), ("EEC1_ENGINE_00", {"ENGINE_SPEED": 1000}),
              ("EEC2_ENGINE_00", {"ACCELERATOR_PEDAL_POSITION_1": gas}),
              ("ETC2_03", {"TRANSMISSION_SELECTED_GEAR": gear, "TRANSMISSION_CURRENT_GEAR": gear if current_gear is None else current_gear})]
    messages = [self.packer.make_can_msg(name, 0, fields) for name, fields in values if name not in omit]
    if ssc_present and self.CP.safetyConfigs[0].safetyParam:
      messages.append(self.ssc_message(ssc_status, corrupt_ssc))
    self.last_messages = messages
    self.nanos += 10_000_000
    state, fp = self.CI.update(car.CarControl.new_message().as_reader(), [can_event(messages, self.nanos)], self.toggles)
    return state, self.aol.update(state, fp, self.sm)

  def warmup(self, **kwargs):
    for _ in range(30):
      state, fp = self.step(**kwargs)
    assert state.canValid
    return state, fp

  def command(self, lat=True, enabled=False, steer=1.0):
    control = car.CarControl.new_message(enabled=enabled, latActive=lat)
    control.actuators.steer = steer
    return self.CI.apply(control.as_reader(), self.nanos, self.toggles)


def test_recorded_cruise_bytes_decode_buttons_and_set_speed():
  parser = CANParser("j1939_standard", [("CCVS_00", 0), ("CCVS_17", 0)], 0)
  # Actual SET and RESUME frames from route 00000010--1412ebded8.
  for switches, expected in (("ffffffc705fffff0", "SET"), ("ffffffc750fffff0", "RESUME")):
    parser.update_strings([can_event([[0x18FEF117, 0, bytes.fromhex(switches), 0],
                                      [0x18FEF100, 0, bytes.fromhex("ff0032fdff31a0c0"), 0]], 1_000_000_000)])
    assert parser.vl["CCVS_17"][f"CRUISE_CONTROL_{expected}_SWITCH"] == 1
    assert parser.vl["CCVS_17"]["CRUISE_CONTROL_SET_SPEED"] == 255
    assert parser.vl["CCVS_00"]["CRUISE_CONTROL_SET_SPEED"] == 49


def test_engagement_test_mode_needs_no_ssc_and_never_sends_commands():
  harness = Harness()
  assert harness.CP.safetyConfigs[0].safetyParam == 0
  assert not harness.CP.openpilotLongitudinalControl
  state, fp = harness.warmup(ssc_present=False)
  assert state.cruiseState.available and not state.cruiseState.enabled
  assert not state.steerFaultTemporary
  assert fp.alwaysOnLateralEnabled
  for lat in (False, True):
    for enabled in (False, True):
      output, sends = harness.command(lat, enabled)
      assert not sends
      assert output.steer == output.accel == output.gas == 0


def test_on_set_cancel_resume_off_and_manual_pedals_keep_independent_steering():
  harness = Harness()
  harness.warmup(main=0)
  sequence = [(1, 0, 0, 0), (1, 1, 0, 0), (1, 0, 0, 0), (1, 0, 1, 0),
              (1, 0, 0, 30), (1, 1, 0, 0), (1, 0, 0, 0), (0, 0, 0, 0)]
  for main, cruise, brake, gas in sequence:
    state, fp = harness.step(main=main, cruise=cruise, brake=brake, gas=gas)
    assert state.cruiseState.available == bool(main)
    assert state.cruiseState.enabled == bool(cruise)
    assert fp.alwaysOnLateralEnabled == bool(main)
    assert not harness.command(lat=fp.alwaysOnLateralEnabled, enabled=bool(cruise))[1]


@pytest.mark.parametrize("switches,button", [(0x05, car.CarState.ButtonEvent.Type.decelCruise),
                                           (0x50, car.CarState.ButtonEvent.Type.accelCruise)])
def test_switch_edges_are_reported_and_set_speed_uses_engine_source(switches, button):
  harness = Harness()
  harness.warmup()
  state, _ = harness.step(switches=switches, set_speed=55)
  assert [(event.type, event.pressed) for event in state.buttonEvents] == [(button, True)]
  assert state.cruiseState.speed == pytest.approx(55 / 3.6)
  state, _ = harness.step()
  assert [(event.type, event.pressed) for event in state.buttonEvents] == [(button, False)]
  state, _ = harness.step(set_speed=255)
  assert state.cruiseState.speed == 0


@pytest.mark.parametrize("change", [{"main": 2}, {"main": 3}, {"switches": 0x55}, {"switches": 0xFF},
                                    {"cruise": 2}, {"cruise": 3}, {"speed": 255.996}, {"brake": 3}])
def test_invalid_or_shutdown_states_do_not_authorize_steering(change):
  harness = Harness()
  harness.warmup()
  state, fp = harness.step(**change)
  assert not state.cruiseState.available
  assert not fp.alwaysOnLateralEnabled


def test_missing_master_clears_steering_and_soft_disables_include_driver_monitoring():
  harness = Harness()
  harness.warmup()
  harness.sm["onroadEvents"] = [SimpleNamespace(softDisable=True, immediateDisable=False, name=car.CarEvent.EventName.driverDistracted)]
  assert not harness.step()[1].alwaysOnLateralEnabled
  harness.sm["onroadEvents"] = []
  assert harness.step()[1].alwaysOnLateralEnabled
  for _ in range(120):
    state, fp = harness.step(omit=("CCVS_17",))
  assert not state.canValid
  assert not fp.alwaysOnLateralEnabled


def test_ssc_requires_feedback_and_steers_when_only_lateral_is_active():
  harness = Harness(ssc=True)
  state, fp = harness.warmup()
  assert not state.steerFaultTemporary
  assert state.steeringAngleDeg == 12.5
  assert state.steeringRateDeg == 90
  assert state.steeringTorqueEps == .5
  assert fp.alwaysOnLateralEnabled and not state.cruiseState.enabled
  decoder = CANParser("hyundai_i30_2014", [("STEERING_COMMAND", 0)], 1)
  for _ in range(12):
    output, sends = harness.command(enabled=False)
    decoder.update_strings([can_event(sends, harness.nanos)])
  assert output.steer > 0
  assert decoder.vl["STEERING_COMMAND"]["STEER_MODE"] == 1
  assert decoder.vl["STEERING_COMMAND"]["STEER_TORQUE"] > 0
  output, sends = harness.command(lat=False)
  decoder.update_strings([can_event(sends, harness.nanos)])
  assert output.steer == 0
  assert decoder.vl["STEERING_COMMAND"]["STEER_MODE"] == 0
  assert decoder.vl["STEERING_COMMAND"]["STEER_TORQUE"] == decoder.vl["STEERING_COMMAND"]["STEER_ANGLE"] == 0


@pytest.mark.parametrize("change", [{"ssc_status": 0}, {"ssc_status": 6}, {"corrupt_ssc": True}])
def test_ssc_status_faults_block_lateral_and_send_hard_off(change):
  harness = Harness(ssc=True)
  harness.warmup()
  state, fp = harness.step(**change)
  assert state.steerFaultTemporary
  assert not fp.alwaysOnLateralEnabled
  output, sends = harness.command()
  assert output.steer == 0
  assert (sends[0][2][1] >> 4) & 3 == 0


def test_ssc_disconnect_blocks_output_and_nonfinite_requests_are_off():
  harness = Harness(ssc=True)
  harness.warmup()
  for _ in range(120):
    state, fp = harness.step(ssc_present=False)
  assert state.steerFaultTemporary and not fp.alwaysOnLateralEnabled
  assert harness.command()[0].steer == 0
  harness.warmup()
  for value in (float("nan"), float("inf"), -float("inf")):
    output, sends = harness.command(steer=value)
    assert output.steer == 0
    assert (sends[0][2][1] >> 4) & 3 == 0


def test_batched_ssc_status_and_counter_faults():
  harness = Harness(ssc=True)
  harness.warmup()
  messages = [harness.ssc_message(), harness.ssc_message()]
  harness.nanos += 10_000_000
  harness.CI.cp_cam.update_strings([can_event(messages, harness.nanos)])
  state, _ = harness.CI.CS.update(harness.CI.cp, harness.CI.cp_cam, harness.toggles)
  assert not state.steerFaultTemporary
  # A repeated valid-checksum frame still fails the status sequence check.
  harness.nanos += 10_000_000
  harness.CI.cp_cam.update_strings([can_event(messages[-1:], harness.nanos)])
  state, _ = harness.CI.CS.update(harness.CI.cp, harness.CI.cp_cam, harness.toggles)
  assert state.steerFaultTemporary


@pytest.mark.parametrize("selected,current,expected", [(-1, -1, "reverse"), (0, 0, "neutral"), (126, 0, "park"),
                                                       (129, 6, "unknown"), (130, 6, "unknown"), (6, -1, "unknown"), (6, 0, "unknown")])
def test_nonforward_and_invalid_transmission_states_stop_ssc(selected, current, expected):
  harness = Harness(ssc=True)
  harness.warmup()
  state, fp = harness.step(gear=selected, current_gear=current)
  assert state.gearShifter == getattr(car.CarState.GearShifter, expected)
  assert not fp.alwaysOnLateralEnabled
  output, sends = harness.command()
  assert output.steer == 0
  assert (sends[0][2][1] >> 4) & 3 == 0


def test_recorded_transmission_frames_decode_drive_reverse_and_neutral():
  parser = CANParser("j1939_standard", [("ETC2_03", 0)], 0)
  for raw, selected, current in (("8181058020362033", 4, 3), ("7ca2137c20522052", -1, -1), ("7d00007d204e204e", 0, 0)):
    parser.update_strings([can_event([[0x18F00503, 0, bytes.fromhex(raw), 0]], 1_000_000_000)])
    assert parser.vl["ETC2_03"]["TRANSMISSION_SELECTED_GEAR"] == selected
    assert parser.vl["ETC2_03"]["TRANSMISSION_CURRENT_GEAR"] == current


def test_controller_commands_pass_native_panda_limits_through_ramps_and_disengagement():
  harness = Harness(ssc=True)
  harness.warmup()
  safety = libpanda_py.libpanda
  safety.set_safety_hooks(Panda.SAFETY_MOTORHOME, 1)
  safety.init_tests()
  safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL)
  for target in (1.0, -1.0, 0.0, 1.0):
    for _ in range(200):
      state, fp = harness.step()
      safety.set_timer((harness.nanos - 1_000_000_000) // 1000)
      for addr, _, data, bus in harness.last_messages:
        assert safety.safety_rx_hook(libpanda_py.make_CANPacket(addr, bus, data))
      output, sends = harness.command(lat=fp.alwaysOnLateralEnabled, enabled=False, steer=target)
      assert abs(output.steer) <= 1
      for addr, _, data, bus in sends:
        assert safety.safety_tx_hook(libpanda_py.make_CANPacket(addr, bus, data)), data.hex()
  harness.step(main=0)
  safety.set_timer((harness.nanos - 1_000_000_000) // 1000)
  for addr, _, data, bus in harness.last_messages:
    safety.safety_rx_hook(libpanda_py.make_CANPacket(addr, bus, data))
  for addr, _, data, bus in harness.command(lat=False)[1]:
    assert safety.safety_tx_hook(libpanda_py.make_CANPacket(addr, bus, data))
