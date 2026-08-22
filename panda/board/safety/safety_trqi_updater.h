#pragma once

// Dedicated OFFROAD service mode for the TRQI CAN bootloader. This is not a
// general diagnostic or arbitrary-output mode. pandad is responsible for
// selecting it only while the TRQI update lease is active.

#define TRQI_UPDATER_BUS 1
#define TRQI_ENTER_BOOT_ADDR 0x60AU
#define TRQI_BOOT_REQUEST_ADDR 0x6A0U

const CanMsg TRQI_UPDATER_TX_MSGS[] = {
  {TRQI_ENTER_BOOT_ADDR, TRQI_UPDATER_BUS, 8},
  {TRQI_BOOT_REQUEST_ADDR, TRQI_UPDATER_BUS, 8},
};

static uint8_t trqi_updater_crc8_poly07(const CANPacket_t *msg, int length) {
  uint8_t crc = 0U;
  for (int i = 0; i < length; i++) {
    crc ^= GET_BYTE(msg, i);
    for (int bit = 0; bit < 8; bit++) {
      crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
    }
  }
  return crc;
}

static safety_config trqi_updater_init(uint16_t param) {
  UNUSED(param);
  controls_allowed = false;
  return (safety_config){NULL, 0, TRQI_UPDATER_TX_MSGS,
                         sizeof(TRQI_UPDATER_TX_MSGS) / sizeof(TRQI_UPDATER_TX_MSGS[0])};
}

static bool trqi_updater_tx_hook(const CANPacket_t *to_send) {
  const uint32_t addr = GET_ADDR(to_send);
  const int bus = (int)GET_BUS(to_send);
  const int length = (int)GET_LEN(to_send);
  bool valid = (bus == TRQI_UPDATER_BUS) && (length == 8);

  if (valid && (addr == TRQI_ENTER_BOOT_ADDR)) {
    const uint32_t nonce = GET_BYTES(to_send, 2, 4);
    valid = (GET_BYTE(to_send, 0) == 0xB0U) &&
            (GET_BYTE(to_send, 1) == 1U) &&
            (nonce != 0U) &&
            (GET_BYTE(to_send, 6) == 0xA5U) &&
            (trqi_updater_crc8_poly07(to_send, 7) == GET_BYTE(to_send, 7));
  } else if (valid && (addr == TRQI_BOOT_REQUEST_ADDR)) {
    const uint8_t pci_type = GET_BYTE(to_send, 0) >> 4U;
    const uint8_t sf_length = GET_BYTE(to_send, 0) & 0x0FU;
    const uint16_t ff_length = ((uint16_t)sf_length << 8U) | GET_BYTE(to_send, 1);
    valid = ((pci_type == 0U) && (sf_length > 0U) && (sf_length <= 7U)) ||
            ((pci_type == 1U) && (ff_length > 6U) && (ff_length <= 600U)) ||
            (pci_type == 2U);
  } else {
    valid = false;
  }

  return valid;
}

const safety_hooks trqi_updater_hooks = {
  .init = trqi_updater_init,
  .rx = default_rx_hook,
  .tx = trqi_updater_tx_hook,
  .fwd = default_fwd_hook,
};
