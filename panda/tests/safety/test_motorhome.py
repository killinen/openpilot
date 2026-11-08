#!/usr/bin/env python3
import unittest

from panda import Panda, ALTERNATIVE_EXPERIENCE
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common


class TestMotorhomeSafety(common.PandaSafetyTest):
  TX_MSGS = [[0x22E, 1]]
  CRUISE_STATUS = 0x18FEF100
  SSC_ENABLED = False

  def setUp(self):
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_MOTORHOME, int(self.SSC_ENABLED))
    self.safety.init_tests()
    self.status_counter = 0
    self._rx(self._master_msg())
    self._rx(self._brake_msg())
    self._rx(self._gear_msg())
    if self.SSC_ENABLED:
      self._rx(self._ssc_status())

  @staticmethod
  def _cruise_status_msg(state: int, bus: int = 0, addr: int = CRUISE_STATUS):
    dat = bytearray(b"\xFF" * 8)
    dat[3] = (dat[3] & 0xFC) | (state & 0x3)
    dat[1:3] = (50 * 256).to_bytes(2, "little")
    return common.make_msg(bus, addr, 8, bytes(dat))

  @staticmethod
  def _master_msg(state=1, switches=0, brake=0, bus=0, length=8):
    dat = bytes([255, 255, 255, 0xC3 | (state << 2) | (brake << 4), switches, 255, 255, 240])
    return common.make_msg(bus, 0x18FEF117, length, dat[:length])

  @staticmethod
  def _brake_msg(brake=0):
    return common.make_msg(0, 0x18FEF121, 8, bytes([243, 255, 255, 0xCF | (brake << 4), 255, 255, 255, 255]))

  @staticmethod
  def _gear_msg(selected=131, current=131):
    return common.make_msg(0, 0x18F00503, 8, bytes([selected, 0, 0, current, 255, 255, 255, 255]))

  @staticmethod
  def _checksum(data, addr):
    total = addr + sum(data)
    return ((total & 255) + (total >> 8)) & 255

  def _ssc_status(self, status=2, corrupt=False, counter=None):
    counter = self.status_counter if counter is None else counter
    self.status_counter = (counter + 1) % 16
    dat = bytearray([0, counter | (status << 4), 0, 0, 80, 0, 0, 0])
    dat[0] = self._checksum(dat[1:7], 0x22F) ^ int(corrupt)
    return common.make_msg(1, 0x22F, 8, bytes(dat))

  def _steer_command(self, bus=1, length=5, mode=0, torque=0, angle=0, corrupt=False):
    dat = bytearray([0, mode << 4, angle & 255, (angle >> 8) & 255, torque & 255])
    dat[0] = self._checksum(dat[1:], 0x22E) ^ int(corrupt)
    return common.make_msg(bus, 0x22E, length, bytes(dat) + bytes(max(0, length - 5)))

  def test_cruise_controls_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._cruise_status_msg(0)))
    self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._cruise_status_msg(1)))
    self.assertTrue(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._cruise_status_msg(0)))
    self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self._cruise_status_msg(1)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_invalid_cruise_states_disable_controls(self):
    for state in (0, 2, 3):
      with self.subTest(state=state):
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._rx(self._cruise_status_msg(state)))
        self.assertFalse(self.safety.get_controls_allowed())

  def test_cruise_status_wrong_bus_or_address_has_no_effect(self):
    for name, msg in (
      ("wrong bus", self._cruise_status_msg(1, bus=1)),
      ("wrong address", self._cruise_status_msg(1, addr=self.CRUISE_STATUS + 1)),
    ):
      with self.subTest(case=name):
        self.assertTrue(self._rx(msg))
        self.assertFalse(self.safety.get_controls_allowed())

  def test_output_setting_gates_actuator_commands(self):
    self.assertEqual(self.SSC_ENABLED, self._tx(self._steer_command()))
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.SSC_ENABLED, self._tx(self._steer_command()))
    if not self.SSC_ENABLED:
      self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))

    self.assertFalse(self._tx(self._steer_command(bus=0)))
    self.assertFalse(self._tx(self._steer_command(length=8)))

  def test_cruise_status_rx_check(self):
    self.assertTrue(self._rx(self._cruise_status_msg(0)))
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

    self.safety.set_timer(2_000_001)
    self.safety.set_controls_allowed(True)
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.safety_config_valid())
    self.assertFalse(self.safety.get_controls_allowed())

  def test_master_off_and_invalid_states_clear_controls(self):
    for state in (0, 2, 3):
      self.safety.set_controls_allowed(True)
      self._rx(self._master_msg(state))
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertFalse(self.safety.get_acc_main_on())

  def test_shutdown_and_invalid_switch_patterns_are_not_master_on(self):
    for switches in (0x55, 0xAA, 0xFF, 0x02):
      self.safety.set_controls_allowed(True)
      self._rx(self._master_msg(switches=switches))
      self.assertFalse(self.safety.get_acc_main_on())
      self.assertFalse(self.safety.get_controls_allowed())


class TestMotorhomeSSCSafety(TestMotorhomeSafety):
  SSC_ENABLED = True

  def enable_aol(self):
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL)
    self._rx(self._cruise_status_msg(0))

  def test_master_allows_independent_steering_across_cancel_and_resume(self):
    self.enable_aol()
    for cruise in (0, 1, 0, 1, 0):
      self._rx(self._cruise_status_msg(cruise))
      self.assertEqual(bool(cruise), self.safety.get_controls_allowed())
      self.assertTrue(self._tx(self._steer_command(mode=1, torque=1)))
    self._rx(self._master_msg(0))
    self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))
    self.assertTrue(self._tx(self._steer_command()))

  def test_cruise_required_without_aol(self):
    self._rx(self._cruise_status_msg(0))
    self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))
    self._rx(self._cruise_status_msg(1))
    self.assertTrue(self._tx(self._steer_command(mode=1, torque=1)))

  def test_missing_status_or_missing_master_blocks_aol(self):
    self.enable_aol()
    for timer in (250001, 500001):
      self.safety.set_timer(timer)
      self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))
      self.assertTrue(self._tx(self._steer_command()))
    # Fresh SSC cannot substitute for a missing master/engine/brake stream.
    self._rx(self._ssc_status())
    self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))

  def test_each_required_coach_stream_has_its_own_timeout(self):
    for missing in ("engine", "master", "brake", "gear"):
      with self.subTest(missing=missing):
        self.setUp()
        self.enable_aol()
        self.safety.set_timer(500001)
        messages = {"engine": self._cruise_status_msg(0), "master": self._master_msg(), "brake": self._brake_msg(), "gear": self._gear_msg()}
        for name, message in messages.items():
          if name != missing:
            self._rx(message)
        self._rx(self._ssc_status())
        self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))

  def test_wrong_bus_or_length_cannot_reenable_master(self):
    self.enable_aol()
    self._rx(self._master_msg(0))
    for message in (self._master_msg(bus=1), self._master_msg(length=5)):
      self._rx(message)
      self.assertFalse(self.safety.get_acc_main_on())
      self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))

  def test_reverse_neutral_park_and_invalid_gear_block_steering(self):
    for selected, current in ((124, 124), (125, 125), (251, 251), (254, 254), (255, 255), (124, 131), (131, 124), (131, 125)):
      with self.subTest(selected=selected, current=current):
        self.setUp()
        self.enable_aol()
        self._rx(self._gear_msg(selected, current))
        self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))
        self.assertTrue(self._tx(self._steer_command()))

  def test_bad_ssc_checksum_counter_and_feedback_state_block_aol(self):
    for kwargs in ({"corrupt": True}, {"status": 0}, {"status": 6}):
      self.setUp()
      self.enable_aol()
      self.assertFalse(self._rx(self._ssc_status(**kwargs)))
      self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))
      self.assertTrue(self._tx(self._steer_command()))
    self.setUp()
    self.enable_aol()
    for _ in range(6):
      self._rx(self._ssc_status(counter=0))
    self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))

  def test_command_checksum_mode_angle_and_torque_limits(self):
    self.enable_aol()
    for kwargs in ({"corrupt": True}, {"mode": 2}, {"mode": 3}, {"angle": 1},
                   {"torque": 33}, {"torque": -33}, {"torque": 2}, {"mode": 0, "torque": 1}):
      command = {"mode": 1, "torque": 1, **kwargs}
      self.assertFalse(self._tx(self._steer_command(**command)))

  def test_invalid_speed_cruise_brake_and_shutdown_block_aol(self):
    for message in (self._cruise_status_msg(2), self._brake_msg(3), self._master_msg(switches=0x55)):
      self.setUp()
      self.enable_aol()
      self._rx(message)
      self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))
    for speed in (0, 65535):
      self.setUp()
      self.enable_aol()
      dat = bytearray(b"\xFF" * 8)
      dat[1:3], dat[3] = speed.to_bytes(2, "little"), 0xFC
      self._rx(common.make_msg(0, self.CRUISE_STATUS, 8, bytes(dat)))
      self.assertFalse(self._tx(self._steer_command(mode=1, torque=1)))


if __name__ == "__main__":
  unittest.main()
