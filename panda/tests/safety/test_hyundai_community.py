#!/usr/bin/env python3
import unittest
from dataclasses import dataclass
from typing import cast

from panda import Panda
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common


def _crc8_pedal(data: bytes) -> int:
  crc = 0xFF
  poly = 0xD5
  for i in range(len(data) - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc


def _i30_compute_checksum(addr: int, dat: bytes) -> int:
  if addr in (0x165, 0x2B0):
    data_length = 5 if addr == 0x2B0 else 7
    chksum = 0
    for i in range(data_length):
      b = dat[i]
      if addr == 0x2B0 and i == 4:
        b &= 0x0F
      chksum ^= b
    if addr == 0x2B0:
      chksum = ((chksum & 0x0F) ^ (chksum >> 4)) & 0xFF
    return chksum

  if addr in (0x200, 0x201):
    return _crc8_pedal(dat[:5])

  if addr == 0x22F:
    ssc_chksum = 0x22F
    for i in range(1, 7):
      ssc_chksum += dat[i]
    ssc_chksum = ((ssc_chksum & 0xFF) + (ssc_chksum >> 8)) & 0xFFFF
    return ssc_chksum & 0xFF

  chksum = 0
  for i in range(8):
    b = dat[i]
    if addr == 0x260 and i == 7:
      b &= 0xF0
    if addr == 0x081 and i == 7:
      b &= 0x0F
    chksum += (b % 16) + (b // 16)
  return (16 - (chksum % 16)) % 16


@dataclass
class _I30Counters:
  # Default all counters to 0 so the first sent counter=1 is "correct" (expected last+1).
  c081: int = 0
  c165: int = 0
  c22f: int = 0
  c201: int = 0
  c260: int = 0
  c2b0: int = 0
  c200: int = 0


class _I30MsgFactory:
  def __init__(self):
    self.c = _I30Counters()

  def _next(self, name: str, max_counter: int) -> int:
    current = cast(int, getattr(self.c, name))
    v = (current + 1) % (max_counter + 1)
    setattr(self.c, name, v)
    return v

  def msg_081(self, brake_pressed: bool) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 8)
    if brake_pressed:
      dat[0] |= 0x80
    cnt = self._next("c081", 15)
    dat[7] = cnt & 0x0F  # low nibble
    chksum = _i30_compute_checksum(0x081, bytes(dat))
    dat[7] = (chksum << 4) | (dat[7] & 0x0F)
    return common.make_msg(0, 0x081, 8, bytes(dat))

  def msg_165(self) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 8)
    cnt = self._next("c165", 15)
    dat[6] = cnt & 0x0F
    dat[7] = _i30_compute_checksum(0x165, bytes(dat)) & 0xFF
    return common.make_msg(0, 0x165, 8, bytes(dat))

  def msg_2b0(self) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 5)
    cnt = self._next("c2b0", 15)
    dat[4] = cnt & 0x0F
    chksum = _i30_compute_checksum(0x2B0, bytes(dat))
    dat[4] = ((chksum & 0x0F) << 4) | (dat[4] & 0x0F)
    return common.make_msg(0, 0x2B0, 5, bytes(dat))

  def msg_22f(self, bus: int = 1) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 8)
    cnt = self._next("c22f", 15)
    dat[1] = (dat[1] & 0xF0) | (cnt & 0x0F)
    dat[0] = _i30_compute_checksum(0x22F, bytes(dat)) & 0xFF
    return common.make_msg(bus, 0x22F, 8, bytes(dat))

  def msg_1f1(self, moving: bool) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 8)
    # i30 uses (speed_fl + speed_rr) * 2 > 30 to determine moving.
    speed = 20 if moving else 0
    speed_fl = speed
    speed_rr = speed
    dat[2] = speed_fl & 0xFF
    dat[3] = (speed_fl >> 8) & 0x0F
    dat[6] = (speed_rr << 4) & 0xF0
    dat[7] = (speed_rr >> 4) & 0xFF
    return common.make_msg(0, 0x1F1, 8, bytes(dat))

  def msg_260(self, cruise_engaged: bool, cruise_main: bool, gas_pressed: bool) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 8)
    if cruise_engaged:
      dat[3] |= 1 << 2
    if cruise_main:
      dat[3] |= 1 << 1

    # Counter lives in bits 4-5 of byte 7, gas pressed in bit 6.
    cnt = self._next("c260", 3) & 0x3
    upper = (cnt << 4) & 0x30
    if gas_pressed:
      upper |= 0x40
    dat[7] = upper

    chksum = _i30_compute_checksum(0x260, bytes(dat))
    dat[7] = (dat[7] & 0xF0) | (chksum & 0x0F)
    return common.make_msg(0, 0x260, 8, bytes(dat))

  def msg_201(self, interceptor_raw: int, bus: int = 1) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 6)
    dat[0] = (interceptor_raw >> 8) & 0xFF
    dat[1] = interceptor_raw & 0xFF
    dat[2] = (interceptor_raw >> 8) & 0xFF
    dat[3] = interceptor_raw & 0xFF
    cnt = self._next("c201", 15)
    dat[4] = cnt & 0x0F
    dat[5] = _i30_compute_checksum(0x201, bytes(dat)) & 0xFF
    return common.make_msg(bus, 0x201, 6, bytes(dat))

  def msg_4f0(self, button: int, main_button: bool = False) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 8)
    dat[0] = button & 0x7
    if main_button:
      dat[3] |= 1 << 0
    return common.make_msg(0, 0x4F0, 8, bytes(dat))

  def msg_200_gas_cmd(self, enable: bool, gas_command: int, gas_command2: int, counter: int | None = None) -> libpanda_py.CANPacket:
    dat = bytearray(b"\x00" * 6)
    dat[0] = (gas_command >> 8) & 0xFF
    dat[1] = gas_command & 0xFF
    dat[2] = (gas_command2 >> 8) & 0xFF
    dat[3] = gas_command2 & 0xFF

    if counter is None:
      counter = self._next("c200", 15)

    dat[4] = (counter & 0x0F) | (0x80 if enable else 0x00)
    dat[5] = _i30_compute_checksum(0x200, bytes(dat)) & 0xFF
    return common.make_msg(1, 0x200, 6, bytes(dat))


class TestHyundaiCommunityI30Lateral(common.PandaSafetyTestBase):
  TX_MSGS = [(0x22E, 1, 5)]

  def setUp(self):
    self.packer = _I30MsgFactory()
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI_COMMUNITY, 0)
    self.safety.init_tests()

  def test_cruise_engage_controls_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self.packer.msg_260(cruise_engaged=True, cruise_main=False, gas_pressed=False)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self.packer.msg_260(cruise_engaged=False, cruise_main=False, gas_pressed=False)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_gas_pressed_disables_controls(self):
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._rx(self.packer.msg_260(cruise_engaged=True, cruise_main=False, gas_pressed=False)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self.packer.msg_260(cruise_engaged=True, cruise_main=False, gas_pressed=True)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_checks_cover_all_checksum_modes(self):
    self.assertTrue(self._rx(self.packer.msg_081(brake_pressed=False)))
    self.assertTrue(self._rx(self.packer.msg_165()))
    self.assertTrue(self._rx(self.packer.msg_2b0()))
    self.assertTrue(self._rx(self.packer.msg_1f1(moving=False)))
    self.assertTrue(self._rx(self.packer.msg_1f1(moving=True)))
    self.assertTrue(self._rx(self.packer.msg_22f()))

  def test_gas_command_blocked_in_lateral_mode(self):
    msg = self.packer.msg_200_gas_cmd(enable=False, gas_command=0, gas_command2=0)
    self.assertFalse(self._tx(msg))


class TestHyundaiCommunityI30Longitudinal(common.PandaSafetyTestBase):
  TX_MSGS = [(0x200, 1, 6), (0x22E, 1, 5)]
  I30_LONGITUDINAL_PARAM = 4

  def setUp(self):
    self.packer = _I30MsgFactory()
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_HYUNDAI_COMMUNITY, self.I30_LONGITUDINAL_PARAM)
    self.safety.init_tests()

  def test_button_enable_disable(self):
    self.assertTrue(self._rx(self.packer.msg_4f0(button=0, main_button=False)))
    self.assertTrue(self._rx(self.packer.msg_4f0(button=1, main_button=False)))
    self.assertTrue(self._rx(self.packer.msg_4f0(button=0, main_button=False)))
    self.assertTrue(self.safety.get_controls_allowed())

    self.assertTrue(self._rx(self.packer.msg_4f0(button=4, main_button=False)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_stock_cruise_main_disables_controls(self):
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._rx(self.packer.msg_260(cruise_engaged=False, cruise_main=True, gas_pressed=False)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_gas_interceptor_sets_gas_pressed(self):
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._rx(self.packer.msg_201(interceptor_raw=750)))
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self.assertTrue(self.safety.get_controls_allowed())

    self.safety.set_controls_allowed(1)
    self.assertTrue(self._rx(self.packer.msg_201(interceptor_raw=2000)))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_controls_allowed())

  def test_gas_command_counter_checksum_and_limits(self):
    self.assertTrue(self._rx(self.packer.msg_260(cruise_engaged=False, cruise_main=False, gas_pressed=False)))
    self.assertTrue(self._rx(self.packer.msg_4f0(button=1, main_button=False)))
    self.assertTrue(self._rx(self.packer.msg_4f0(button=0, main_button=False)))
    self.assertTrue(self.safety.get_controls_allowed())

    neutral = self.packer.msg_200_gas_cmd(enable=False, gas_command=0, gas_command2=0, counter=0)
    self.assertTrue(self._tx(neutral))

    enabled = self.packer.msg_200_gas_cmd(enable=True, gas_command=1000, gas_command2=1000, counter=1)
    self.assertTrue(self._tx(enabled))

    bad_gas_command2_range = self.packer.msg_200_gas_cmd(enable=True, gas_command=1000, gas_command2=2000, counter=2)
    self.assertFalse(self._tx(bad_gas_command2_range))

    self.safety.set_controls_allowed(0)
    blocked_no_controls = self.packer.msg_200_gas_cmd(enable=True, gas_command=1000, gas_command2=1000, counter=2)
    self.assertFalse(self._tx(blocked_no_controls))
    self.safety.set_controls_allowed(1)

    bad_counter = self.packer.msg_200_gas_cmd(enable=True, gas_command=1000, gas_command2=1000, counter=4)
    self.assertFalse(self._tx(bad_counter))

    bad_neutral = self.packer.msg_200_gas_cmd(enable=False, gas_command=1, gas_command2=0, counter=2)
    self.assertFalse(self._tx(bad_neutral))

    dat = bytearray(b"\x00" * 6)
    dat[0] = (1000 >> 8) & 0xFF
    dat[1] = 1000 & 0xFF
    dat[2] = (1000 >> 8) & 0xFF
    dat[3] = 1000 & 0xFF
    dat[4] = (2 & 0x0F) | 0x80
    dat[5] = (_i30_compute_checksum(0x200, bytes(dat)) ^ 0xFF) & 0xFF
    bad_crc = common.make_msg(1, 0x200, 6, bytes(dat))
    self.assertFalse(self._tx(bad_crc))

    bad_range = self.packer.msg_200_gas_cmd(enable=True, gas_command=3000, gas_command2=1000, counter=2)
    self.assertFalse(self._tx(bad_range))

  def test_fwd_hook_blocks(self):
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x123), -1)


if __name__ == "__main__":
  unittest.main()
