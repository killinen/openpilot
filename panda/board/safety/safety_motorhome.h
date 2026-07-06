const CanMsg MOTORHOME_TX_MSGS[] = {
  {0x22E, 1, 5},  // SSC steering command on actuator bus
};

static safety_config motorhome_init(uint16_t param) {
  UNUSED(param);
  controls_allowed = true;
  return (safety_config){NULL, 0, MOTORHOME_TX_MSGS, sizeof(MOTORHOME_TX_MSGS) / sizeof(MOTORHOME_TX_MSGS[0])};
}

static bool motorhome_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return controls_allowed;
}

const safety_hooks motorhome_hooks = {
  .init = motorhome_init,
  .rx = default_rx_hook,
  .tx = motorhome_tx_hook,
  .fwd = default_fwd_hook,
};
