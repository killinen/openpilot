const int I30_MAX_STEER = 384;             // like stock
const int I30_MAX_RT_DELTA = 112;          // max delta torque allowed for real time checks
const uint32_t I30_RT_INTERVAL = 250000;   // 250ms between real time checks
const int I30_MAX_RATE_UP = 3;
const int I30_MAX_RATE_DOWN = 7;
const int I30_DRIVER_TORQUE_ALLOWANCE = 50;
const int I30_DRIVER_TORQUE_FACTOR = 2;
const int I30_STANDSTILL_THRSLD = 30;  // ~1kph

const int I30_MAX_ACCEL = 200;  // 1/100 m/s2
const int I30_MIN_ACCEL = -350; // 1/100 m/s2
const int I30_TRQI_MAX_TORQUE_NCM = 1001;   // match the openpilot-side TRQI steering demand window

#define I30_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2U) // avg between 2 tracks

// These are messages that will/can be sent to the busses
const CanMsg I30_TX_MSGS[] = {
  {0x22E, 1, 5},  // SSC Bus 1
  {0x232, 1, 7},  // TRQI_TorqueCmd Bus 1
};

const CanMsg I30_LONG_TX_MSGS[] = {
  {0x200, 1, 6},  // GAS_COMMAND Bus 1
  {0x22E, 1, 5},  // SSC Bus 1
  {0x232, 1, 7},  // TRQI_TorqueCmd Bus 1
};

// older i30 models have less checks due to missing counters and checksums
const int I30_PARAM_LONGITUDINAL = 4;

enum {
  I30_BTN_NONE = 0,
  I30_BTN_RESUME = 1,
  I30_BTN_SET = 2,
  I30_BTN_CANCEL = 4,
};

// some newer HKG models can re-enable after spamming cancel button,
// so keep track of user button presses to deny engagement if no interaction
const uint8_t I30_PREV_BUTTON_SAMPLES = 8;  // roughly 160 ms
static uint8_t i30_last_button_interaction;  // button messages since the user pressed an enable button
static bool i30_longitudinal = false;
static bool i30_stock_cruise_main = false;
static uint8_t i30_counter_pedal_last = 0xFFU;
static uint8_t i30_counter_trqi_last = 0xFFU;
static bool i30_gas_interceptor_detected = false;

RxCheck i30_rx_checks[] = {
  {.msg = {{0x081, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 83U}, { 0 }, { 0 }}},   // EMS_DCT2 (129)
  {.msg = {{0x165, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 83U}, { 0 }, { 0 }}},   // VSM2 (357)
  {.msg = {{0x1F1, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 50U}, { 0 }, { 0 }}},   // TCS5 (497)
  {.msg = {{0x260, 0, 8, .check_checksum = true, .max_counter = 3U, .frequency = 83U}, { 0 }, { 0 }}},    // EMS6 (608)
  {.msg = {{0x2B0, 0, 5, .check_checksum = true, .max_counter = 15U, .frequency = 83U}, { 0 }, { 0 }}},   // SAS1 (688)
  {.msg = {{0x22F, 1, 8, .check_checksum = true, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},  // SSC (559, bus 1)
};

RxCheck i30_long_rx_checks[] = {
  {.msg = {{0x081, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 83U}, { 0 }, { 0 }}},   // EMS_DCT2 (129)
  {.msg = {{0x165, 0, 8, .check_checksum = true, .max_counter = 15U, .frequency = 83U}, { 0 }, { 0 }}},   // VSM2 (357)
  {.msg = {{0x1F1, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 50U}, { 0 }, { 0 }}},   // TCS5 (497)
  {.msg = {{0x260, 0, 8, .check_checksum = true, .max_counter = 3U, .frequency = 83U}, { 0 }, { 0 }}},    // EMS6 (608)
  {.msg = {{0x2B0, 0, 5, .check_checksum = true, .max_counter = 15U, .frequency = 83U}, { 0 }, { 0 }}},   // SAS1 (688)
  {.msg = {{0x22F, 1, 8, .check_checksum = false, .max_counter = 15U, .frequency = 100U}, { 0 }, { 0 }}},  // SSC (559)
  {.msg = {{0x201, 1, 6, .check_checksum = true, .max_counter = 15U, .frequency = 50U}, { 0 }, { 0 }}},   // Gas interceptor (513, bus 1)
};

static uint8_t crc8_pedal(const uint8_t *data, int len) {
  uint8_t crc = 0xFFU;
  const uint8_t poly = 0xD5U;
  for (int i = len - 1; i >= 0; i--) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if ((((uint32_t)crc) & ((uint32_t)0x80U)) != ((uint32_t)0U)) {
        crc = (uint8_t)(((uint8_t)(crc << 1)) ^ poly);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

static uint8_t i30_compute_pedal_crc(const CANPacket_t *to_push) {
  uint8_t dat[5];
  for (int i = 0; i < 5; i++) {
    dat[i] = GET_BYTE(to_push, i);
  }
  return crc8_pedal(dat, 5);
}

static uint8_t i30_compute_trqi_crc8(const CANPacket_t *to_push) {
  uint8_t crc = 0x00U;
  const uint8_t addr_bytes[2] = {
    (uint8_t)(GET_ADDR(to_push) & 0xFFU),
    (uint8_t)((GET_ADDR(to_push) >> 8) & 0xFFU),
  };

  for (int i = 0; i < 2; i++) {
    crc ^= addr_bytes[i];
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ 0x07U);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    crc ^= GET_BYTE(to_push, i);
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ 0x07U);
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }

  return crc;
}

static uint8_t i30_get_counter(const CANPacket_t *to_push) {
  const uint32_t addr = GET_ADDR(to_push);

  uint8_t cnt = 0U;
  if (addr == 0x081U) {
    cnt = GET_BYTE(to_push, 7) & 0xFU;
  } else if (addr == 0x165U) {
    cnt = GET_BYTE(to_push, 6) & 0xFU;
  } else if (addr == 0x22FU) {
    cnt = GET_BYTE(to_push, 1) & 0xFU;
  } else if (addr == 0x231U) {
    cnt = GET_BYTE(to_push, 5) & 0xFU;
  } else if (addr == 0x232U) {
    cnt = GET_BYTE(to_push, 5) & 0xFU;
  } else if (addr == 0x201U) {
    cnt = GET_BYTE(to_push, 4) & 0xFU;
  } else if (addr == 0x260U) {
    cnt = (GET_BYTE(to_push, 7) >> 4) & 0x3U;
  } else if (addr == 0x2B0U) {
    cnt = GET_BYTE(to_push, 4) & 0xFU;
  } else {
  }
  return cnt;
}

static uint32_t i30_get_checksum(const CANPacket_t *to_push) {
  const uint32_t addr = GET_ADDR(to_push);

  uint8_t chksum = 0U;
  if (addr == 0x081U) {
    chksum = (GET_BYTE(to_push, 7) >> 4) & 0xFU;
  } else if (addr == 0x165U) {
    chksum = GET_BYTE(to_push, 7);
  } else if (addr == 0x201U) {
    chksum = GET_BYTE(to_push, 5);
  } else if (addr == 0x231U) {
    chksum = GET_BYTE(to_push, 6);
  } else if (addr == 0x232U) {
    chksum = GET_BYTE(to_push, 6);
  } else if (addr == 0x22FU) {
    chksum = GET_BYTE(to_push, 0);
  } else if (addr == 0x260U) {
    chksum = GET_BYTE(to_push, 7) & 0xFU;
  } else if (addr == 0x2B0U) {
    chksum = (GET_BYTE(to_push, 4) >> 4) & 0xFU;
  } else {
  }
  return chksum;
}

static uint32_t i30_compute_checksum(const CANPacket_t *to_push) {
  const uint32_t addr = GET_ADDR(to_push);

  uint8_t chksum = 0U;
  if ((addr == 0x165U) || (addr == 0x2B0U)) {
    const int data_length = (addr == 0x2B0U) ? 5 : 7;
    for (int i = 0; i < data_length; i++) {
      uint8_t b = GET_BYTE(to_push, i);
      if ((addr == 0x2B0U) && (i == 4)) {
        b &= 0x0FU;
      }
      chksum ^= b;
    }
    if (addr == 0x2B0U) {
      const uint32_t tmp = (uint32_t)chksum;
      // cppcheck-suppress misra-c2012-10.4 ; tooling false-positive on explicit unsigned operations
      chksum = (uint8_t)((tmp & (uint32_t)0x0FU) ^ (tmp >> (uint32_t)4U));
    }
  } else if (addr == 0x201U) {
    chksum = i30_compute_pedal_crc(to_push);
  } else if ((addr == 0x231U) || (addr == 0x232U)) {
    chksum = i30_compute_trqi_crc8(to_push);
  } else if (addr == 0x22FU) {
    uint16_t ssc_chksum = (uint16_t)0x22FU;
    const int data_length = 7;
    for (int i = 1; i < data_length; i++) {
      ssc_chksum += GET_BYTE(to_push, i);
    }
    ssc_chksum = (uint16_t)((ssc_chksum & 0xFFU) + (ssc_chksum >> 8));
    chksum = (uint8_t)(ssc_chksum & 0xFFU);
  } else {
    const int data_length = 8;
    for (int i = 0; i < data_length; i++) {
      uint8_t b = GET_BYTE(to_push, i);
      if (((addr == 0x260U) && (i == 7)) || ((addr == 0x081U) && (i == 7))) {
        b &= (addr == 0x081U) ? 0x0FU : 0xF0U;
      }
      chksum += (b % 16U) + (b / 16U);
    }
    chksum = (16U - (chksum % 16U)) % 16U;
  }

  return chksum;
}

static int16_t i30_decode_signed12_low_word(uint8_t lo, uint8_t hi) {
  uint16_t raw12 = (uint16_t)lo | (((uint16_t)hi & 0x0FU) << 8U);

  if ((raw12 & 0x0800U) != 0U) {
    raw12 |= 0xF000U;
  }

  return (int16_t)raw12;
}

static uint16_t i30_decode_raw12_low_word(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | (((uint16_t)hi & 0x0FU) << 8U);
}

static void i30_rx_hook(const CANPacket_t *to_push) {
  const int bus = GET_BUS(to_push);
  const int addr = GET_ADDR(to_push);

  const bool bus0 = bus == 0;
  const bool bus1_interceptor = (bus == 1) && (addr == 0x201);

  if (bus0 || bus1_interceptor) {
    // ACC steering wheel buttons (addr 0x4F0 on i30)
    if (addr == 0x4F0) {
      const int cruise_button = GET_BYTE(to_push, 0) & 0x7U;
      const bool main_button = GET_BIT(to_push, 24U);

      if ((cruise_button == I30_BTN_RESUME) || (cruise_button == I30_BTN_SET) || (cruise_button == I30_BTN_CANCEL) || main_button) {
        i30_last_button_interaction = 0U;
      } else {
        i30_last_button_interaction = MIN(i30_last_button_interaction + 1U, I30_PREV_BUTTON_SAMPLES);
      }

      if (i30_longitudinal) {
        if (cruise_button == I30_BTN_CANCEL) {
          controls_allowed = false;
        }

        // enter controls on falling edge of resume or set
        const bool set = (cruise_button == I30_BTN_NONE) && (cruise_button_prev == I30_BTN_SET);
        const bool res = (cruise_button == I30_BTN_NONE) && (cruise_button_prev == I30_BTN_RESUME);
        if (set || res) {
          if (!i30_stock_cruise_main) {
            controls_allowed = true;
          }
        }

        cruise_button_prev = cruise_button;
      }
    }

    // Lateral-only: enter controls when stock cruise engages, exit when disengaged
    if ((!i30_longitudinal) && (addr == 0x260)) {
      const bool cruise_engaged = (((GET_BYTE(to_push, 3) >> 2) & 0x1U) != 0U);
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = true;
      }
      if (!cruise_engaged) {
        controls_allowed = false;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // check for gas interceptor
    if (addr == 0x201) {
      i30_gas_interceptor_detected = true;
      const int gas_interceptor = I30_GET_INTERCEPTOR(to_push);
      const int I30_GAS_INTERCEPTOR_THRESHOLD = 1100; // avg of raw vals at 0 gas is 750
      gas_pressed = gas_interceptor > I30_GAS_INTERCEPTOR_THRESHOLD;
    }

    // read gas pressed signal
    if (!i30_gas_interceptor_detected) {
      if (addr == 0x260) {  // ICE
        gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0U;
      }
    }

    // sample wheel speed, averaging opposite corners
    if (addr == 0x1F1) {
      const uint32_t speed_fl = (GET_BYTES(to_push, 0, 4) >> 16U) & 0xFFFU;
      const uint32_t speed_rr = (GET_BYTES(to_push, 4, 4) >> 20U) & 0xFFFU;
      const uint32_t speed_scaled = (speed_fl + speed_rr) * 2U;
      vehicle_moving = speed_scaled > (uint32_t)I30_STANDSTILL_THRSLD;
    }

    if (addr == 0x081) {
      brake_pressed = (GET_BYTE(to_push, 0) >> 7) != 0U;
    }

    bool stock_ecu_detected = false;

    // For i30 longitudinal (pedal interceptor), don't allow openpilot controls while stock cruise is in the MAIN (standby) state.
    if (i30_longitudinal && (addr == 0x260)) {
      i30_stock_cruise_main = (((GET_BYTE(to_push, 3) >> 1) & 0x1U) != 0U);
      if (i30_stock_cruise_main) {
        controls_allowed = false;
      }
    }

    generic_rx_checks(stock_ecu_detected);
  }
}

static bool i30_tx_hook(const CANPacket_t *to_send) {
  const int addr = GET_ADDR(to_send);

  bool tx = true;

  // TRQI desired EPS output torque command on 0x232:
  // signed low-12-bit Ncm value in bytes0..1, raw12 ones-complement in bytes2..3, relay flags in byte 4,
  // 4-bit rolling counter in byte 5, CRC-8 in byte 6.
  if (addr == 0x232) {
    const uint16_t torque_raw12 = i30_decode_raw12_low_word(GET_BYTE(to_send, 0), GET_BYTE(to_send, 1));
    const uint16_t torque_complement_raw12 = i30_decode_raw12_low_word(GET_BYTE(to_send, 2), GET_BYTE(to_send, 3));
    const int16_t torque = i30_decode_signed12_low_word(GET_BYTE(to_send, 0), GET_BYTE(to_send, 1));
    const uint8_t flags = GET_BYTE(to_send, 4);
    const bool rel_cmd = (flags & 0x1U) != 0U;
    const bool rele_cmd = (flags & 0x2U) != 0U;
    const uint8_t counter = GET_BYTE(to_send, 5) & 0xFU;
    const uint8_t checksum = GET_BYTE(to_send, 6);

    bool violation = false;
    const bool complement_valid = ((torque_raw12 ^ 0x0FFFU) == torque_complement_raw12);
    const bool neutral = (torque == 0) && complement_valid && !rel_cmd && !rele_cmd;
    const bool counter_valid = (((i30_counter_trqi_last + 1U) & 0xFU) == counter);
    const bool counter_initted = (i30_counter_trqi_last != 0xFFU);

    if (!complement_valid) {
      violation = true;
    }
    if ((torque < -I30_TRQI_MAX_TORQUE_NCM) || (torque > I30_TRQI_MAX_TORQUE_NCM)) {
      violation = true;
    }
    if (i30_compute_checksum(to_send) != checksum) {
      violation = true;
    }
    if ((!counter_initted || !counter_valid) && !neutral) {
      violation = true;
    }
    if (!controls_allowed && !neutral) {
      violation = true;
    }

    if (violation) {
      tx = false;
    } else if (tx) {
      i30_counter_trqi_last = counter;
    } else {
    }
  }

  // GAS Pedal Interceptor command
  if (addr == 0x200) {
    const bool enable = GET_BIT(to_send, 39U);
    const int gas_command = (GET_BYTE(to_send, 0) << 8) | GET_BYTE(to_send, 1);
    const int gas_command2 = (GET_BYTE(to_send, 2) << 8) | GET_BYTE(to_send, 3);
    const uint8_t counter = GET_BYTE(to_send, 4) & 0xFU;
    const uint8_t checksum = GET_BYTE(to_send, 5);

    bool violation = false;
    const int GAS_COMMAND_MIN = 0;
    const int GAS_COMMAND_MAX = 2700;
    const int GAS_COMMAND2_MIN = 0;
    const int GAS_COMMAND2_MAX = 1300;

    if (!i30_longitudinal) {
      violation = true;
    } else if (enable) {
      if ((gas_command < GAS_COMMAND_MIN) || (gas_command > GAS_COMMAND_MAX)) {
        violation = true;
      }
      if ((gas_command2 < GAS_COMMAND2_MIN) || (gas_command2 > GAS_COMMAND2_MAX)) {
        violation = true;
      }
      if (!get_longitudinal_allowed()) {
        violation = true;
      }
    } else {
      if ((gas_command != GAS_COMMAND_MIN) || (gas_command2 != GAS_COMMAND2_MIN)) {
        violation = true;
      }
    }

    const bool neutral = !enable && (gas_command == GAS_COMMAND_MIN) && (gas_command2 == GAS_COMMAND2_MIN);
    const bool counter_valid = (((i30_counter_pedal_last + 1U) & 0xFU) == counter);
    const bool counter_initted = (i30_counter_pedal_last != 0xFFU);
    if ((!counter_initted || !counter_valid) && !neutral) {
      violation = true;
    }

    if (i30_compute_pedal_crc(to_send) != checksum) {
      violation = true;
    }

    if (violation) {
      tx = false;
    } else if (tx) {
      i30_counter_pedal_last = counter;
    } else {
    }
  }

  return tx;
}

static int i30_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  // Mirror selected vehicle-state frames onto the actuator bus for TRQI/auxiliary consumers on bus 1.
  if ((bus_num == 0) && ((addr == 0x081) || (addr == 0x165) || (addr == 0x1F1) || (addr == 0x329))) {
    bus_fwd = 1;
  }

  return bus_fwd;
}

static safety_config i30_init(uint16_t param) {
  i30_last_button_interaction = I30_PREV_BUTTON_SAMPLES;
  i30_longitudinal = GET_FLAG(param, I30_PARAM_LONGITUDINAL);
  i30_stock_cruise_main = false;
  i30_counter_pedal_last = 0xFFU;
  i30_counter_trqi_last = 0xFFU;
  i30_gas_interceptor_detected = false;

  safety_config ret = BUILD_SAFETY_CFG(i30_rx_checks, I30_TX_MSGS);
  if (i30_longitudinal) {
    ret = BUILD_SAFETY_CFG(i30_long_rx_checks, I30_LONG_TX_MSGS);
  } else {
  }
  return ret;
}

const safety_hooks i30_hooks = {
  .init = i30_init,
  .rx = i30_rx_hook,
  .tx = i30_tx_hook,
  .fwd = i30_fwd_hook,
  .get_checksum = i30_get_checksum,
  .compute_checksum = i30_compute_checksum,
  .get_counter = i30_get_counter,
};
