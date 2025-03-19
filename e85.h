#ifndef E85HEADER
#define E85HEADER

#include <stdint.h>
#include "esp32-hal-log.h"
#include "hal/twai_types.h"

// CAN arbitration IDs for known messages.
static constexpr uint32_t can_asc1_id = 0x153;
static constexpr uint32_t can_asc2_id = 0x1F0;
static constexpr uint32_t can_asc3_id = 0x1F3;
static constexpr uint32_t can_asc4_id = 0x1F8;
static constexpr uint32_t can_lws1_id = 0x1F5;
static constexpr uint32_t can_dme1_id = 0x316;
static constexpr uint32_t can_dme2_id = 0x329;
static constexpr uint32_t can_dme3_id = 0x338;
static constexpr uint32_t can_dme4_id = 0x545;
static constexpr uint32_t can_icl3_id = 0x615;

// Desired update frequencies for a given message. In Hz (1/s).
static constexpr uint32_t can_asc1_freq = 20;  // 10ms(ASC)/20ms(DSC) native
static constexpr uint32_t can_asc2_freq = 20;  // 10ms(ASC)/20ms(DSC) native
static constexpr uint32_t can_asc3_freq = 10;  // 20ms native
static constexpr uint32_t can_asc4_freq = 20;  // 20ms native
static constexpr uint32_t can_lws1_freq = 10;  // 10ms native
static constexpr uint32_t can_dme1_freq = 20;  // 10ms native
static constexpr uint32_t can_dme2_freq = 10;  // 10ms native
static constexpr uint32_t can_dme3_freq = 1;   // 1000ms native
static constexpr uint32_t can_dme4_freq = 1;   // 10ms native
static constexpr uint32_t can_icl3_freq = 1;   // 200ms native
static constexpr uint32_t can_default_freq = 1;


struct CanData {
  uint8_t accel_pos;  // DME2
  uint8_t air_temp;
  uint8_t air_pressure;
  bool brake_switch;
  uint8_t brake_pressure;
  bool clutch_switch;
  uint8_t coolant_temp;
  uint16_t engine_rpm;
  uint8_t oil_temp;
  uint16_t speed;
  uint16_t steering_angle;
};

static void packCanMessage(const struct CanData& data, uint8_t* buf) {
  buf[0] = (data.speed >> 8) & 0xFF;
  buf[1] = data.speed & 0xFF;
  buf[2] = data.brake_pressure;
  buf[3] = (data.steering_angle >> 8) & 0xFF;
  buf[4] = data.steering_angle & 0xFF;
  buf[5] = (data.engine_rpm >> 8) & 0xFF;
  buf[6] = data.engine_rpm & 0xFF;
  buf[7] = data.clutch_switch;
  buf[8] = data.coolant_temp;
  buf[9] = data.air_pressure;
  buf[10] = data.accel_pos;
  buf[11] = data.oil_temp;
  buf[12] = data.air_temp;
}

static bool handleCanMessage(const twai_message_t& message, struct CanData& data) {
  switch (message.identifier) {
    case can_asc1_id:
      data.speed = (message.data[1] << 8) | message.data[2];
      break;
    case can_asc2_id:
      // 4 wheel speed sensors
      break;
    case can_asc3_id:
      // X and Y acceleration
      break;
    case can_asc4_id:
      data.brake_pressure = message.data[2];
      break;
    case can_lws1_id:
      data.steering_angle = (message.data[0] << 8) | message.data[1];
      break;
    case can_dme1_id:
      data.engine_rpm = (message.data[2] << 8) | message.data[3];
      break;
    case can_dme2_id:
      data.coolant_temp = message.data[1];
      data.air_pressure = message.data[2];
      data.clutch_switch = message.data[3] & 0x1;
      data.accel_pos = message.data[5];
      data.brake_switch = message.data[6] & 0x1;
      break;
    case can_dme3_id:
      // Sport button
      break;
    case can_dme4_id:
      data.oil_temp = message.data[4];
      break;
    case can_icl3_id:
      data.air_temp = message.data[3];
      break;
    default:
      break;
  }

  if (message.identifier == can_lws1_id) {
    return true;
  } else {
    return false;
  }
}

static uint16_t get_notify_interval_ms(uint32_t pid) {
  switch (pid) {
    case can_asc1_id: return 1000 / can_asc1_freq;
    case can_asc2_id: return 1000 / can_asc2_freq;
    case can_asc3_id: return 1000 / can_asc3_freq;
    case can_asc4_id: return 1000 / can_asc4_freq;
    case can_lws1_id: return 1000 / can_lws1_freq;
    case can_dme1_id: return 1000 / can_dme1_freq;
    case can_dme2_id: return 1000 / can_dme2_freq;
    case can_dme3_id: return 1000 / can_dme3_freq;
    case can_dme4_id: return 1000 / can_dme4_freq;
    case can_icl3_id: return 1000 / can_icl3_freq;
    default: return 1000 / can_default_freq;
  }
}

static bool canPidAllowed2(uint32_t pid) {
  switch (pid) {
    case can_asc1_id:
    case can_asc2_id:
    case can_asc3_id:
    case can_asc4_id:
    case can_lws1_id:
    case can_dme1_id:
    case can_dme2_id:
    case can_dme3_id:
    case can_dme4_id:
    case can_icl3_id:
      return true;
  }

  return false;
}

static bool canPidAllowed(uint32_t pid) {
  switch (pid) {
    case can_asc1_id:
      {
        static uint16_t asc1_count = 0;
        ++asc1_count;
        return (asc1_count & 0x1) == 0;
      }
    case can_asc2_id:
      {
        static uint16_t asc2_count = 0;
        ++asc2_count;
        return (asc2_count & 0x1) == 0;
      }
    case can_asc3_id:
      {
        static uint16_t asc3_count = 0;
        ++asc3_count;
        return (asc3_count & 0x1) == 0;
      }
    case can_asc4_id:
      {
        static uint16_t asc4_count = 0;
        ++asc4_count;
        return (asc4_count & 0x1) == 0;
      }
    case can_lws1_id:
      {
        static uint16_t lws1_count = 0;
        ++lws1_count;
        return (lws1_count & 0x1) == 0;
      }
    case can_dme1_id:
      {
        static uint16_t dme1_count = 0;
        ++dme1_count;
        return (dme1_count & 0x1) == 0;
      }
    case can_dme2_id:
      {
        static uint16_t dme2_count = 0;
        ++dme2_count;
        return (dme2_count & 0x1) == 0;
      }
    case can_dme3_id:
      {
        static uint16_t dme3_count = 0;
        ++dme3_count;
        return (dme3_count & 0x1) == 0;
      }
    case can_dme4_id:
      {
        static uint16_t dme4_count = 0;
        ++dme4_count;
        return (dme4_count & 0x1) == 0;
      }
    case can_icl3_id:
      {
        static uint16_t icl3_count = 0;
        ++icl3_count;
        return (icl3_count & 0x1) == 0;
      }
      return true;
  }

  return false;
}

static void get_can_asc1_msg2(twai_message_t* msg, int speed_kmh) {
  uint16_t s = speed_kmh * 16;
  msg->flags = 0;
  msg->identifier = can_asc1_id;
  msg->data_length_code = 8;
  msg->data[0] = 0;
  msg->data[1] = s & 0xff;
  msg->data[2] = (s >> 8) & 0xff;
  msg->data[3] = 0;
  msg->data[4] = 0;
  msg->data[5] = 0;
  msg->data[6] = 0;
  msg->data[7] = 0;
}

static void get_can_lws1_msg2(twai_message_t* msg, int steering_angle_deg) {
  int16_t s = steering_angle_deg / 0.04394;
  msg->flags = 0;
  msg->identifier = can_lws1_id;
  msg->data_length_code = 8;
  msg->data[0] = s >> 8;
  msg->data[1] = s & 0xff;
  msg->data[2] = 0;
  msg->data[3] = 0;
  msg->data[4] = 0;
  msg->data[5] = 0;
  msg->data[6] = 0;
  msg->data[7] = 0;
}

static void get_can_dme1_msg(twai_message_t* msg, int rpm) {
  uint16_t s = rpm * 6.4;
  msg->flags = 0;
  msg->identifier = can_dme1_id;
  msg->data_length_code = 8;
  msg->data[0] = 0;
  msg->data[1] = 0;
  msg->data[2] = s & 0xff;
  msg->data[3] = (s >> 8) & 0xff;
  msg->data[4] = 0;
  msg->data[5] = 0;
  msg->data[6] = 0;
  msg->data[7] = 0;
}


static void get_can_dme2_msg(twai_message_t* msg, int eng_temp_water_c, int ambient_hPa, int tps_perc) {
  msg->flags = 0;
  msg->identifier = can_dme2_id;
  msg->data_length_code = 8;
  msg->data[0] = 0;
  msg->data[1] = ((eng_temp_water_c + 48) / 0.75) /* + 1*/;  // +1 might not be correct.
  msg->data[2] = (ambient_hPa - 598) / 2;
  msg->data[3] = 0;
  msg->data[4] = 0;
  msg->data[5] = max(0x01, min((int)(tps_perc * 2.56), 0xfe));
  msg->data[6] = 0;
  msg->data[7] = 0;
}

static void get_can_dme4_msg(twai_message_t* msg, int eng_temp_oil_c) {
  msg->flags = 0;
  msg->identifier = can_dme4_id;
  msg->data_length_code = 8;
  msg->data[0] = 0;
  msg->data[1] = 0;
  msg->data[2] = 0;
  msg->data[3] = 0;
  msg->data[4] = eng_temp_oil_c + 48;
  msg->data[5] = 0;
  msg->data[6] = 0;
  msg->data[7] = 0;
}

static void get_can_icl3_msg(twai_message_t* msg, int ambient_temp_c) {
  msg->flags = 0;
  msg->identifier = can_icl3_id;
  msg->data_length_code = 8;
  msg->data[0] = 0;
  msg->data[1] = 0;
  msg->data[2] = 0;
  msg->data[3] = ambient_temp_c;
  msg->data[4] = 0;
  msg->data[5] = 0;
  msg->data[6] = 0;
  msg->data[7] = 0;
}

#endif
