#!/usr/bin/env python3
import unittest

from panda import Panda
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common


class TestMotorhomeSafety(common.PandaSafetyTest):
  TX_MSGS = [[0x22E, 1]]
  CRUISE_STATUS = 0x18FEF100

  def setUp(self):
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_MOTORHOME, 0)
    self.safety.init_tests()

  @staticmethod
  def _cruise_status_msg(state: int, bus: int = 0, addr: int = CRUISE_STATUS):
    dat = bytearray(b"\xFF" * 8)
    dat[3] = (dat[3] & 0xFC) | (state & 0x3)
    return common.make_msg(bus, addr, 8, bytes(dat))

  @staticmethod
  def _steer_command(bus: int = 1, length: int = 5):
    return common.make_msg(bus, 0x22E, length)

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

  def test_steer_command_requires_controls_allowed(self):
    self.assertFalse(self._tx(self._steer_command()))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._steer_command()))

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


if __name__ == "__main__":
  unittest.main()
