#!/usr/bin/env python3
import unittest

from panda import Panda
from panda.tests.libpanda import libpanda_py


def crc8_poly07(data: bytes) -> int:
  crc = 0
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


def enter_boot(nonce: int = 0x12345678) -> bytes:
  data = bytes((0xB0, 1)) + nonce.to_bytes(4, "little") + bytes((0xA5,))
  return data + bytes((crc8_poly07(data),))


class TestTrqiUpdaterSafety(unittest.TestCase):
  def setUp(self):
    self.safety = libpanda_py.libpanda
    self.assertEqual(0, self.safety.set_safety_hooks(Panda.SAFETY_TRQI_UPDATER, 0))
    self.safety.init_tests()

  def tx(self, address: int, data: bytes, bus: int = 1) -> bool:
    return bool(self.safety.safety_tx_hook(libpanda_py.make_CANPacket(address, bus, data)))

  def test_only_exact_ids_bus_and_length(self):
    self.assertTrue(self.tx(0x60A, enter_boot()))
    self.assertTrue(self.tx(0x6A0, bytes.fromhex("0122000000000000")))
    self.assertFalse(self.tx(0x60B, enter_boot()))
    self.assertFalse(self.tx(0x60A, enter_boot(), bus=2))
    self.assertFalse(self.tx(0x60A, enter_boot()[:-1]))

  def test_entry_validation(self):
    valid = bytearray(enter_boot())
    for index in (0, 1, 2, 6, 7):
      corrupt = valid.copy()
      corrupt[index] ^= 1
      self.assertFalse(self.tx(0x60A, bytes(corrupt)))
    self.assertFalse(self.tx(0x60A, enter_boot(0)))

  def test_isotp_pci_validation(self):
    self.assertTrue(self.tx(0x6A0, bytes.fromhex("0737000000000000")))
    self.assertTrue(self.tx(0x6A0, bytes.fromhex("10AABB0000000000")))
    self.assertTrue(self.tx(0x6A0, bytes.fromhex("2100000000000000")))
    self.assertFalse(self.tx(0x6A0, bytes.fromhex("0000000000000000")))
    self.assertFalse(self.tx(0x6A0, bytes.fromhex("1006000000000000")))
    self.assertFalse(self.tx(0x6A0, bytes.fromhex("1259000000000000")))
    self.assertFalse(self.tx(0x6A0, bytes.fromhex("3000000000000000")))

  def test_controls_never_allowed(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_controls_allowed(True)
    self._reset_mode()
    self.assertFalse(self.safety.get_controls_allowed())

  def _reset_mode(self):
    self.assertEqual(0, self.safety.set_safety_hooks(Panda.SAFETY_TRQI_UPDATER, 0))


if __name__ == "__main__":
  unittest.main()
