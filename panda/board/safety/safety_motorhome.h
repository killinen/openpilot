const int MOTORHOME_CRUISE_STATUS = 0x18FEF100;

const CanMsg MOTORHOME_TX_MSGS[] = {
  {0x22E, 1, 5},  // SSC steering command on actuator bus
};

RxCheck motorhome_rx_checks[] = {
  {.msg = {{MOTORHOME_CRUISE_STATUS, 0, 8, .frequency = 5U}, { 0 }, { 0 }}},
};

static safety_config motorhome_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(motorhome_rx_checks, MOTORHOME_TX_MSGS);
}

static void motorhome_rx_hook(const CANPacket_t *to_push) {
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if ((bus == 0) && (addr == MOTORHOME_CRUISE_STATUS)) {
    // SPN 595 is the low two bits of byte 3: 0 = off, 1 = on, 2 = error, 3 = not available.
    const bool cruise_engaged = (GET_BYTE(to_push, 3) & 0x3U) == 1U;
    pcm_cruise_check(cruise_engaged);
  }
}

static bool motorhome_tx_hook(const CANPacket_t *to_send) {
  UNUSED(to_send);
  return controls_allowed;
}

const safety_hooks motorhome_hooks = {
  .init = motorhome_init,
  .rx = motorhome_rx_hook,
  .tx = motorhome_tx_hook,
  .fwd = default_fwd_hook,
};
