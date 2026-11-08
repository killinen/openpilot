const int MOTORHOME_CRUISE_STATUS = 0x18FEF100;
const int MOTORHOME_CRUISE_MASTER = 0x18FEF117;
const int MOTORHOME_BRAKE_STATUS = 0x18FEF121;
const int MOTORHOME_TRANSMISSION = 0x18F00503;
const int MOTORHOME_PARAM_SSC = 1;

// SSC torque encoding is 0.125 Nm/count. These match the commissioning
// controller's 4 Nm ceiling, with room for quantization in its 5 Nm/s ramp.
const SteeringLimits MOTORHOME_STEERING_LIMITS = {
  .max_steer = 32, .max_rate_up = 1, .max_rate_down = 2,
  .max_rt_delta = 12, .max_rt_interval = 250000,
  .type = TorqueMotorLimited, .max_torque_error = 32,
};

static bool motorhome_ssc_enabled = false;
static bool motorhome_cruise_valid = false;
static bool motorhome_speed_valid = false;
static bool motorhome_brake_valid = false;
static bool motorhome_forward_gear = false;

const CanMsg MOTORHOME_TX_MSGS[] = {
  {0x22E, 1, 5},  // SSC steering command on actuator bus
};

RxCheck motorhome_rx_checks[] = {
  {.msg = {{MOTORHOME_CRUISE_STATUS, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{MOTORHOME_CRUISE_MASTER, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{MOTORHOME_BRAKE_STATUS, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{MOTORHOME_TRANSMISSION, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{0x22F, 1, 8, .check_checksum = true, .max_counter = 15U, .quality_flag = true, .frequency = 20U}, { 0 }, { 0 }}},
};

static safety_config motorhome_init(uint16_t param) {
  motorhome_ssc_enabled = GET_FLAG(param, MOTORHOME_PARAM_SSC);
  motorhome_cruise_valid = false;
  motorhome_speed_valid = false;
  motorhome_brake_valid = false;
  motorhome_forward_gear = false;
  acc_main_on = false;
  safety_config cfg = BUILD_SAFETY_CFG(motorhome_rx_checks, MOTORHOME_TX_MSGS);
  if (!motorhome_ssc_enabled) {
    cfg.rx_checks_len--;  // Engagement tests must not require an SSC.
  }
  return cfg;
}

static bool motorhome_fresh(int index, uint32_t timeout) {
  return motorhome_rx_checks[index].status.msg_seen && is_msg_valid(motorhome_rx_checks, index) &&
         (get_ts_elapsed(microsecond_timer_get(), motorhome_rx_checks[index].status.last_timestamp) <= timeout);
}

static uint32_t motorhome_compute_checksum(const CANPacket_t *msg) {
  uint16_t sum = (uint16_t)GET_ADDR(msg);
  // Status excludes its final DEBUG_STATES byte; commands use all data bytes.
  const int end = (GET_ADDR(msg) == 0x22FU) ? 7 : (int)GET_LEN(msg);
  for (int i = 1; i < end; i++) {
    sum += GET_BYTE(msg, i);
  }
  sum = (sum & 0xFFU) + (sum >> 8U);
  return sum & 0xFFU;
}

static uint32_t motorhome_get_checksum(const CANPacket_t *msg) {
  return GET_BYTE(msg, 0);
}

static uint8_t motorhome_get_counter(const CANPacket_t *msg) {
  return GET_BYTE(msg, 1) & 0xFU;
}

static bool motorhome_ssc_quality(const CANPacket_t *msg) {
  // Feedback enabled, and no soft-off state. No undocumented DEBUG bits used.
  return ((GET_BYTE(msg, 1) & 0x20U) != 0U) && ((GET_BYTE(msg, 1) & 0x40U) == 0U);
}

static void motorhome_rx_hook(const CANPacket_t *to_push) {
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  if ((bus == 0) && (addr == MOTORHOME_CRUISE_STATUS) && (GET_LEN(to_push) == 8U)) {
    // SPN 595 is the low two bits of byte 3: 0 = off, 1 = on, 2 = error, 3 = not available.
    const bool cruise_engaged = (GET_BYTE(to_push, 3) & 0x3U) == 1U;
    motorhome_cruise_valid = (GET_BYTE(to_push, 3) & 0x3U) <= 1U;
    const int speed = GET_BYTE(to_push, 1) | (GET_BYTE(to_push, 2) << 8);
    motorhome_speed_valid = speed <= 64000;
    vehicle_moving = motorhome_speed_valid && (speed > 256);
    // controls_allowed remains tied to stock cruise for heartbeat/mismatch
    // compatibility. AOL steering permission is separate in the TX hook.
    pcm_cruise_check(cruise_engaged && motorhome_cruise_valid && acc_main_on && motorhome_fresh(1, 500000U));
  }
  if ((bus == 0) && (addr == MOTORHOME_CRUISE_MASTER) && (GET_LEN(to_push) == 8U)) {
    const uint8_t switches = GET_BYTE(to_push, 4);
    bool switches_valid = switches != 0x55U;
    for (uint32_t shift = 0U; shift < 8U; shift += 2U) {
      switches_valid = switches_valid && (((switches >> shift) & 3U) <= 1U);
    }
    const bool brake_valid = ((GET_BYTE(to_push, 3) >> 4U) & 3U) <= 1U;
    acc_main_on = (((GET_BYTE(to_push, 3) >> 2U) & 3U) == 1U) && switches_valid && brake_valid;
    if (!acc_main_on) {
      controls_allowed = false;
      cruise_engaged_prev = false;
    }
  }
  if ((bus == 0) && (addr == MOTORHOME_BRAKE_STATUS) && (GET_LEN(to_push) == 8U)) {
    motorhome_brake_valid = ((GET_BYTE(to_push, 3) >> 4U) & 3U) <= 1U;
  }
  if ((bus == 0) && (addr == MOTORHOME_TRANSMISSION) && (GET_LEN(to_push) == 8U)) {
    // SPNs 524/523, offset -125. Require selected AND current forward gear
    // so selecting Reverse/Neutral disables torque before the shift completes.
    const int selected = GET_BYTE(to_push, 0);
    const int current = GET_BYTE(to_push, 3);
    motorhome_forward_gear = (selected > 125) && (selected <= 250) && (current > 125) && (current <= 250);
  }
  if ((bus == 1) && (addr == 0x22F) && (GET_LEN(to_push) == 8U) && motorhome_ssc_enabled) {
    update_sample(&torque_meas, to_signed(GET_BYTE(to_push, 2), 8));
  }
}

static bool motorhome_tx_hook(const CANPacket_t *to_send) {
  bool tx = false;
  if (motorhome_ssc_enabled && (GET_ADDR(to_send) == 0x22EU) && (GET_BUS(to_send) == 1U) && (GET_LEN(to_send) == 5U)) {
    const int mode = (GET_BYTE(to_send, 1) >> 4U) & 3U;
    const int torque = to_signed(GET_BYTE(to_send, 4), 8);
    const bool format_valid = ((GET_BYTE(to_send, 1) & 0xC0U) == 0U) &&
                             (GET_BYTE(to_send, 2) == 0U) && (GET_BYTE(to_send, 3) == 0U) &&
                             (motorhome_get_checksum(to_send) == motorhome_compute_checksum(to_send));
    if (format_valid) {
      if ((mode == 0) && (torque == 0)) {
        desired_torque_last = 0;
        rt_torque_last = 0;
        ts_torque_check_last = microsecond_timer_get();
        tx = true;  // Always permit a valid hard OFF, including during faults.
      } else {
        const bool ready = acc_main_on && motorhome_cruise_valid && motorhome_speed_valid && motorhome_brake_valid && vehicle_moving && motorhome_forward_gear &&
                           motorhome_fresh(0, 500000U) && motorhome_fresh(1, 500000U) &&
                           motorhome_fresh(2, 500000U) && motorhome_fresh(3, 500000U) && motorhome_fresh(4, 250000U);
        const bool lateral_allowed = controls_allowed || ((alternative_experience & ALT_EXP_ALWAYS_ON_LATERAL) != 0);
        tx = ready && lateral_allowed && (mode == 1) && !steer_torque_cmd_checks(torque, 1, MOTORHOME_STEERING_LIMITS);
      }
    }
  }
  return tx;
}

const safety_hooks motorhome_hooks = {
  .init = motorhome_init,
  .rx = motorhome_rx_hook,
  .tx = motorhome_tx_hook,
  .fwd = default_fwd_hook,
  .get_checksum = motorhome_get_checksum,
  .compute_checksum = motorhome_compute_checksum,
  .get_counter = motorhome_get_counter,
  .get_quality_flag_valid = motorhome_ssc_quality,
};
