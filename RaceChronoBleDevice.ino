#include <Arduino.h>
#include <driver/twai.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "TinyGPS.h"
#include "e85.h"
#include "gps.h"
// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
#include "esp_gatt_common_api.h"

#define CAN_POLLING_RATE_MS 1
#define SERVICE_UUID "00001ff8-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

static const char *TAG = "racechrono_canbus_ble";
bool canBusAllowUnknownPackets = false;
bool isCanBusConnected = false;
bool isBleConnected = false;
uint16_t conn_id = 0;  // Only valid when isBleConnected is true.
BLECharacteristic *cbMainChar = nullptr;
BLECharacteristic *cbGpsMainChar = nullptr;
BLECharacteristic *cbGpsTimeChar = nullptr;
QueueHandle_t xQueueCombined;
TinyGPSPlus gps;
TinyGPSCustom hdop(gps, "GPGSA", 16);
TinyGPSCustom vdop(gps, "GPGSA", 17);

struct xQueueItem {
  enum { invalid = 0,
         twai = 1,
         gps = 2 } type;
  union U {
    twai_message_t twai_message;
    GpsData gps_data;
    U();
  } data;
};

xQueueItem::U::U()
  : gps_data() {
}

// Stats and counters
static uint64_t ble_notify_count = 0;
static uint64_t ble_no_tx_buf_evt_count = 0;
static uint64_t can_rx_count = 0;
static uint64_t can_not_interested_count = 0;
static uint64_t can_queue_enqueue_count = 0;
static uint64_t can_queue_full_count = 0;
static uint64_t gps_line_read_count = 0;
static uint64_t gps_line_invalid_count = 0;
static uint64_t gps_queue_enqueue_count = 0;
static uint64_t gps_queue_full_count = 0;

void stats() {
  static uint64_t last_ble_notify_count = 0;
  static uint64_t last_ble_no_tx_buf_evt_count = 0;
  static uint64_t last_can_rx_count = 0;
  static uint64_t last_can_not_interested_count = 0;
  static uint64_t last_can_queue_enqueue_count = 0;
  static uint64_t last_can_queue_full_count = 0;
  static uint64_t last_gps_line_read_count = 0;
  static uint64_t last_gps_line_invalid_count = 0;

  uint64_t diff_ble_notify_count = ble_notify_count - last_ble_notify_count;
  uint64_t diff_ble_no_tx_buf_evt_count = ble_no_tx_buf_evt_count - last_ble_no_tx_buf_evt_count;
  uint64_t diff_can_rx_count = can_rx_count - last_can_rx_count;
  uint64_t diff_can_not_interested_count = can_not_interested_count - last_can_not_interested_count;
  uint64_t diff_can_queue_enqueue_count = can_queue_enqueue_count - last_can_queue_enqueue_count;
  uint64_t diff_can_queue_full_count = can_queue_full_count - last_can_queue_full_count;
  uint64_t diff_gps_line_read_count = gps_line_read_count - last_gps_line_read_count;
  uint64_t diff_gps_line_invalid_count = gps_line_invalid_count - last_gps_line_invalid_count;

  last_ble_notify_count = ble_notify_count;
  last_ble_no_tx_buf_evt_count = ble_no_tx_buf_evt_count;
  last_can_rx_count = can_rx_count;
  last_can_not_interested_count = can_not_interested_count;
  last_can_queue_enqueue_count = can_queue_enqueue_count;
  last_can_queue_full_count = can_queue_full_count;
  last_gps_line_read_count = gps_line_read_count;
  last_gps_line_invalid_count = gps_line_invalid_count;

  Serial.printf("ble_notify_count/s %llu, ", diff_ble_notify_count);
  Serial.printf("ble_notify_bytes/s %llu, ", diff_ble_notify_count * sizeof(twai_message_t));
  Serial.printf("ble_no_tx_buf_evt_count/s %llu, ", diff_ble_no_tx_buf_evt_count);
  Serial.printf("can_rx_count/s %llu, ", diff_can_rx_count);
  Serial.printf("can_not_interested_count/s %llu, ", diff_can_not_interested_count);
  Serial.printf("can_queue_enqueue_count/s %llu, ", diff_can_queue_enqueue_count);
  Serial.printf("can_queue_full_count/s %llu, ", diff_can_queue_full_count);
  Serial.printf("gps_line_read_count/s %llu, ", diff_gps_line_read_count);
  Serial.printf("gps_line_invalid_count/s %llu, ", diff_gps_line_invalid_count);
  Serial.println("");
}

class MyCanbusFilterCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    static const int CAN_BUS_CMD_DENY_ALL = 0;
    static const int CAN_BUS_CMD_ALLOW_ALL = 1;
    static const int CAN_BUS_CMD_ADD_PID = 2;

    String value = pCharacteristic->getValue();
    if (value.length() < 1) {
      return;
    }
    switch (value[0]) {
      case CAN_BUS_CMD_DENY_ALL:
        {
          if (value.length() == 1) {
            ESP_LOGI(TAG, "CAN-Bus command DENY");
          }
          break;
        }
      case CAN_BUS_CMD_ALLOW_ALL:
        {
          if (value.length() == 3) {
            uint16_t notifyIntervalMs = value[1] << 8 | value[2];
            notifyIntervalMs = 1000;
            canBusAllowUnknownPackets = true;
            ESP_LOGI(TAG, "CAN-Bus command ALLOW interval %d ms", notifyIntervalMs);
          }
          break;
        }
      case CAN_BUS_CMD_ADD_PID:
        {
          if (value.length() == 7) {
            uint16_t notifyIntervalMs = value[1] << 8 | value[2];
            uint32_t pid = value[3] << 24 | value[4] << 16 | value[5] << 8 | value[6];
            notifyIntervalMs = get_notify_interval_ms(pid);
            ESP_LOGI(TAG, "CAN-Bus command ADD PID %d interval %d ms", pid, notifyIntervalMs);
          }
        }
        break;
      default:
        ESP_LOGE(TAG, "Unknown CAN-Bus command 0x%x", value[0]);
        break;
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
    Serial.println("Device connected!");
    ESP_LOGI(TAG, "Device connected!");
    pServer->updateConnParams(param->connect.remote_bda,
                              6,     // Min connection interval: 6 * 1.25ms = 7.5ms
                              6,     // Max connection interval: 6 * 1.25ms = 7.5ms
                              0,     // Latency
                              500);  // Timeout: 500 * 10ms = 5000ms

    conn_id = pServer->getConnId();
    isBleConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected. Start Advertising!");
    ESP_LOGI(TAG, "Device disconnected. Start Advertising!");
    isBleConnected = false;
    conn_id = 0;
    BLEDevice::startAdvertising();
  }
};


void ble_setup() {
  BLEDevice::init("💩💯👌😂 hi!");
  BLEDevice::setMTU(517);
  BLEDevice::setPower(ESP_PWR_LVL_P21);
  BLEDevice::setPower(ESP_PWR_LVL_P21);
  BLEDevice::setPower(ESP_PWR_LVL_P21, ESP_BLE_PWR_TYPE_CONN_HDL0);
  ESP_ERROR_CHECK(esp_ble_gap_set_preferred_default_phy(
    ESP_BLE_GAP_PHY_2M_PREF_MASK,
    ESP_BLE_GAP_PHY_2M_PREF_MASK));
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCanbusMainCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x01), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  cbMainChar = pCanbusMainCharacteristic;

  BLECharacteristic *pCanbusFilterCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x02), BLECharacteristic::PROPERTY_WRITE);
  pCanbusFilterCharacteristic->setCallbacks(new MyCanbusFilterCallbacks());

  BLECharacteristic *pGpsMainCharacteristic = pService->createCharacteristic(
    BLEUUID(uint16_t(0x0003)), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  cbGpsMainChar = pGpsMainCharacteristic;
  // RC is stuck in a "no fix to satellites" state,
  // if the characteristic contains no data on first read.
  uint8_t main_buf[20] = {};
  cbGpsMainChar->setValue(main_buf, sizeof(main_buf));

  BLECharacteristic *pGpsTimeCharacteristic = pService->createCharacteristic(
    BLEUUID(uint16_t(0x0004)), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  cbGpsTimeChar = pGpsTimeCharacteristic;
  // RC is stuck in a "no fix to satellites" state,
  // if the characteristic contains no data on first read.
  uint8_t time_buf[3] = {};
  cbGpsTimeChar->setValue(time_buf, sizeof(time_buf));

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  // play around with lower/higher connection interval values.
  pAdvertising->setMinPreferred(0x06);  // 7.5 ms, minimum connection rate, functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x06);  // 6 * 1.25 ms = 7.5 ms = ~133 Hz
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
  Serial.printf("CONN1 tx power: %d\n", esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_CONN_HDL0));
  Serial.printf("ADV tx power: %d\n", esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV));
  Serial.printf("SCAN tx power: %d\n", esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_SCAN));
  Serial.printf("DEFAULT tx power: %d\n", esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT));
}

void sendCanMsgBle(uint32_t id, uint8_t *data, uint8_t len) {
  if (!isBleConnected) {
    return;
  }
  if (!cbMainChar) {
    return;
  }
  while (esp_ble_get_cur_sendable_packets_num(conn_id) == 0) {
    ++ble_no_tx_buf_evt_count;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  uint8_t buf[20] = {};
  buf[0] = (uint8_t)(id >> 0);
  buf[1] = (uint8_t)(id >> 8);
  buf[2] = (uint8_t)(id >> 16);
  buf[3] = (uint8_t)(id >> 24);
  memcpy(buf + 4, data, std::min(len, (uint8_t)16));
  cbMainChar->setValue(buf, sizeof(id) + std::min(len, (uint8_t)16));
  cbMainChar->notify();
  ++ble_notify_count;
}

void sendGpsMsgBle(struct GpsData &data) {
  if (!isBleConnected) {
    return;
  }
  if (!cbGpsMainChar || !cbGpsTimeChar) {
    return;
  }
  while (esp_ble_get_cur_sendable_packets_num(conn_id) == 0) {
    ++ble_no_tx_buf_evt_count;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  uint8_t buf[20] = {};

  // Sync bits and time from hour start
  uint32_t timeFromHourStart = (data.minutes * 30000) + (data.seconds * 500) + (data.milliseconds / 2);
  buf[0] = ((data.gpsSyncBits & 0x7) << 5) | ((timeFromHourStart >> 16) & 0x1F);
  buf[1] = (timeFromHourStart >> 8) & 0xFF;
  buf[2] = timeFromHourStart & 0xFF;

  // Fix quality and locked satellites
  buf[3] = ((std::min<uint8_t>(0x3, data.fixQuality) & 0x3) << 6) | ((std::min<uint8_t>(0x3F, data.numberOfSatellites)) & 0x3F);

  // Latitude
  buf[4] = (data.latitude >> 24) & 0xFF;
  buf[5] = (data.latitude >> 16) & 0xFF;
  buf[6] = (data.latitude >> 8) & 0xFF;
  buf[7] = data.latitude & 0xFF;

  // Longitude
  buf[8] = (data.longitude >> 24) & 0xFF;
  buf[9] = (data.longitude >> 16) & 0xFF;
  buf[10] = (data.longitude >> 8) & 0xFF;
  buf[11] = data.longitude & 0xFF;

  // Altitude
  buf[12] = (data.altitude >> 8) & 0xFF;
  buf[13] = data.altitude & 0xFF;

  // Speed
  buf[14] = (data.speedOverGround >> 8) & 0xFF;
  buf[15] = data.speedOverGround & 0xFF;

  // Bearing
  buf[16] = (data.courseOverGround >> 8) & 0xFF;
  buf[17] = data.courseOverGround & 0xFF;

  // HDOP
  buf[18] = data.hdop;

  // VDOP
  buf[19] = data.vdop;

  cbGpsMainChar->setValue(buf, sizeof(buf));
  cbGpsMainChar->notify();
  ++ble_notify_count;

  // Update the GPS time characteristic.
  uint8_t time_buf[3] = {};
  time_buf[0] = ((data.gpsSyncBits & 0x7) << 5) | ((data.dateAndHour >> 16) & 0x1F);
  time_buf[1] = (data.dateAndHour >> 8) & 0xFF;
  time_buf[2] = data.dateAndHour & 0xFF;
  cbGpsTimeChar->setValue(time_buf, sizeof(time_buf));
  // No notification needed. RC will read value when required.
  // cbGpsTimeChar->notify();
}

void canBusSetup() {
  // CAN1 setup.
  Serial.println("Initializing builtin CAN peripheral");
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX, (gpio_num_t)CAN1_RX, TWAI_MODE_LISTEN_ONLY /*TWAI_MODE_NORMAL*/);
  // The E85 CAN bus transmits about 927 messages per second. With a slightly
  // longer rx queue, we observed no queue overruns.
  g_config.rx_queue_len = 16;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // TODO: this filter misses icl3 messages.
  // twai_filter_config_t f_config = {
  //   .acceptance_code = ((0x0100 << 3) << 16) | (0x0400 << 3),
  //   .acceptance_mask = 0xF7FFDFFF,
  //   .single_filter = false,
  // };

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN1 Driver initialized");
  } else {
    Serial.println("Failed to initialze CAN1 driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("CAN1 interface started");
  } else {
    Serial.println("Failed to start CAN1");
    return;
  }

  // Disable CAN alerts, as we don't act on them anyway.
  // uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  // if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
  //   Serial.println("CAN1 Alerts reconfigured");
  // } else {
  //   Serial.println("Failed to reconfigure alerts");
  //   return;
  // }

  isCanBusConnected = true;
}

/**
 * @brief Dump a representation of binary data to the console.
 *
 * @param [in] pData Pointer to the start of data to be logged.
 * @param [in] length Length of the data (in bytes) to be logged.
 * @return N/A.
 */
static void hexDump(const uint8_t *pData, uint32_t length) {
  char ascii[80];
  char hex[80];
  char tempBuf[80];
  uint32_t lineNumber = 0;

  ESP_LOGI(TAG, "     00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");
  ESP_LOGI(TAG, "     -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --");
  strcpy(ascii, "");
  strcpy(hex, "");
  uint32_t index = 0;
  while (index < length) {
    sprintf(tempBuf, "%.2x ", pData[index]);
    strcat(hex, tempBuf);
    if (isprint(pData[index])) {
      sprintf(tempBuf, "%c", pData[index]);
    } else {
      sprintf(tempBuf, ".");
    }
    strcat(ascii, tempBuf);
    index++;
    if (index % 16 == 0) {
      ESP_LOGI(TAG, "%.4x %s %s", lineNumber * 16, hex, ascii);
      strcpy(ascii, "");
      strcpy(hex, "");
      lineNumber++;
    }
  }
  if (index % 16 != 0) {
    while (index % 16 != 0) {
      strcat(hex, "   ");
      index++;
    }
    ESP_LOGI(TAG, "%.4x %s %s", lineNumber * 16, hex, ascii);
  }
}  // hexDump

static void dumpTwaiMessage(const twai_message_t &message) {
  Serial.print("CAN1: Received ");
  // Process received message
  if (message.extd) {
    Serial.print("extended ");
  } else {
    Serial.print("standard ");
  }

  if (message.rtr) {
    Serial.print("RTR ");
  }

  Serial.printf("packet with id 0x%x", message.identifier);

  if (message.rtr) {
    Serial.printf(" and requested length %d\n", message.data_length_code);
  } else {
    Serial.printf(" and length %d\n", message.data_length_code);
    Serial.printf("CAN1: Data: %.*s\n", message.data_length_code, message.data);
    hexDump(message.data, message.data_length_code);
  }
}

void taskCanBusLoop(void *) {
  for (;;) {
    // Manage CAN-Bus connection
    if (!isCanBusConnected && isBleConnected) {
      // Connect to CAN-Bus
      Serial.println("Connecting CAN-Bus...");
      if (twai_start() == ESP_OK) {
        isCanBusConnected = true;
        pinMode(LED_BUILTIN, HIGH);
        Serial.println("CAN1 interface started");
      } else {
        Serial.println("Failed to start CAN1");
        delay(3000);
        continue;
      }
    } else if (isCanBusConnected && !isBleConnected) {
      // Disconnect from CAN-Bus
      twai_stop();
      isCanBusConnected = false;
      pinMode(LED_BUILTIN, LOW);
      Serial.println("Stopped CAN1");
    }

    // Handle CAN-Bus data
    if (!isCanBusConnected) {  // TODO: use driver status as flag
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // // check if alert happened
    // uint32_t alerts_triggered;
    // twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(CAN_POLLING_RATE_MS));
    // twai_status_info_t twaistatus;
    // twai_get_status_info(&twaistatus);

    // // Handle alerts
    // if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    //   Serial.println("CAN1: Alert: TWAI controller has become error passive.");
    // }
    // if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    //   Serial.println("CAN1: Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    //   Serial.printf("CAN1: Bus error count: %d\n", twaistatus.bus_error_count);
    // }
    // if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    //   Serial.println("CAN1: Alert: The Transmission failed.");
    //   Serial.printf("CAN1: TX buffered: %d\t", twaistatus.msgs_to_tx);
    //   Serial.printf("CAN1: TX error: %d\t", twaistatus.tx_error_counter);
    //   Serial.printf("CAN1: TX failed: %d\n", twaistatus.tx_failed_count);
    // }
    // if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    //   Serial.println("CAN1: Alert: The RX queue is full causing a received frame to be lost.");
    //   Serial.printf("CAN1: RX buffered: %d\t", twaistatus.msgs_to_rx);
    //   Serial.printf("CAN1: RX missed: %d\t", twaistatus.rx_missed_count);
    //   Serial.printf("CAN1: RX overrun %d\n", twaistatus.rx_overrun_count);
    // }
    // if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
    //   Serial.println("CAN1: Alert: The Transmission was successful.");
    //   Serial.printf("CAN1: TX buffered: %d\n", twaistatus.msgs_to_tx);
    // }
    // // Check if message is received
    // if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    //   // read here
    // }

    xQueueItem combined = {};
    combined.type = xQueueItem::twai;
    while (twai_receive(&combined.data.twai_message, pdMS_TO_TICKS(CAN_POLLING_RATE_MS)) == ESP_OK) {
      ++can_rx_count;
      if (combined.data.twai_message.rtr) {
        ++can_not_interested_count;
        continue;
      }
      if (!canPidAllowed(combined.data.twai_message.identifier)) {
        ++can_not_interested_count;
        continue;
      }
      if (xQueueSend(xQueueCombined, &combined, 0)) {
        ++can_queue_enqueue_count;
      } else {
        ++can_queue_full_count;
      }
    }
  }
}

void taskSendBle(void *) {
  xQueueItem combined;
  for (;;) {
    if (xQueueReceive(xQueueCombined, &combined, pdMS_TO_TICKS(1000))) {
      switch (combined.type) {
        case xQueueItem::twai:
          sendCanMsgBle(combined.data.twai_message.identifier, combined.data.twai_message.data, combined.data.twai_message.data_length_code);
          break;
        case xQueueItem::gps:
          sendGpsMsgBle(combined.data.gps_data);
          break;
      }
    }
  }
}

void taskReadGPS(void *) {
  const uint8_t set_power_save_off[] = { 0xA0, 0xA1, 0x00, 0x03, 0x0C, /*mode*/ 0x00, 0x00, /*CS*/ 0x0C, 0x0D, 0x0A };
  const uint8_t set_baud_rate[] = { 0xA0, 0xA1, 0x00, 0x04, 0x05, 0x00, /*baud*/ 0x05, 0x00, /*CS*/ 0x00, 0x0D, 0x0A };
  const uint8_t set_baud_rate_3[] = { 0xA0, 0xA1, 0x00, 0x04, 0x05, 0x00, /*baud*/ 0x03, 0x00, /*CS*/ 0x06, 0x0D, 0x0A };
  const uint8_t set_baud_rate_perm[] = { 0xA0, 0xA1, 0x00, 0x04, 0x05, 0x00, /*baud*/ 0x05, 0x01, /*CS*/ 0x01, 0x0D, 0x0A };
  const uint8_t set_gps_rate_1[] = { 0xA0, 0xA1, 0x00, 0x03, 0x0E, /*gps*/ 0x01, 0x00, /*CS*/ 0x0F, 0x0D, 0x0A };
  const uint8_t set_gps_rate_4[] = { 0xA0, 0xA1, 0x00, 0x03, 0x0E, /*gps*/ 0x04, 0x00, /*CS*/ 0x0A, 0x0D, 0x0A };
  const uint8_t set_gps_rate_10[] = { 0xA0, 0xA1, 0x00, 0x03, 0x0E, /*gps*/ 0x0A, 0x00, /*CS*/ 0x04, 0x0D, 0x0A };
  const uint8_t set_gps_rate_20[] = { 0xA0, 0xA1, 0x00, 0x03, 0x0E, /*gps*/ 0x14, 0x00, /*CS*/ 0x1A, 0x0D, 0x0A };

  Serial1.begin(9600, SERIAL_8N1, 41, 40);  // Initialize serial communication with the GPS module
  delay(1000);
  while (!Serial1) {
    delay(500);
  }
  Serial1.write(set_power_save_off, sizeof(set_power_save_off));
  delay(500);
  Serial1.write(set_baud_rate_3, sizeof(set_baud_rate_3));
  delay(1000);
  Serial1.begin(38400, SERIAL_8N1, 41, 40);
  delay(1000);
  // Serial1.write(set_gps_rate_20, sizeof(set_gps_rate_20));
  Serial1.write(set_gps_rate_10, sizeof(set_gps_rate_10));
  // Serial1.write(set_gps_rate_4, sizeof(set_gps_rate_4));
  // Serial1.write(set_gps_rate_1, sizeof(set_gps_rate_1));
  delay(1000);

  xQueueItem combined;
  combined.type = xQueueItem::gps;
  GpsData &gps_data = combined.data.gps_data;

  for (;;) {
    if (!isBleConnected) {
      vTaskDelay(500);
      continue;
    }

    while (Serial1.available()) {  // Check if data is available from the GPS module
      int c = Serial1.read();
      if (c == -1) {
        break;
      }
      if (!gps.encode(c)) {
        continue;
      }

      if (gps.time.isUpdated()) {
        gps_data.hours = gps.time.hour();
        gps_data.minutes = gps.time.minute();
        gps_data.seconds = gps.time.second();
        gps_data.milliseconds = gps.time.centisecond() * 10;
      }

      if (gps.date.isUpdated() && gps.time.isValid()) {
        uint32_t dateAndHour = (uint32_t(gps.date.year() - 2000) * 8928) + (uint32_t(gps.date.month() - 1) * 744) + (uint32_t(gps.date.day() - 1) * 24) + gps_data.hours;
        if (gps_data.dateAndHour != dateAndHour) {
          ++gps_data.gpsSyncBits;
          gps_data.dateAndHour = dateAndHour;
        }
      }

      if (gps.satellites.isUpdated()) {
        gps_data.numberOfSatellites = gps.satellites.value();
      }

      if (gps.altitude.isUpdated()) {
        gps_data.altitude = uint16_t((gps.altitude.meters() + 500) * 10.) & 0x7FFF;
      }

      if (gps.speed.isUpdated()) {
        gps_data.speedOverGround = uint16_t(gps.speed.kmph() * 100.) & 0x7FFF;
      }

      if (gps.course.isUpdated()) {
        gps_data.courseOverGround = gps.course.value();
      }

      if (vdop.isUpdated()) {
        gps_data.vdop = atof(vdop.value()) * 10.;
      }

      if (hdop.isUpdated()) {
        gps_data.hdop = atof(hdop.value()) * 10.;
      }

      if (gps.location.isUpdated()) {
        gps_data.fixQuality = char(gps.location.FixQuality()) - '0';
        if (gps_data.fixQuality > 0) {
          gps_data.latitude = gps.location.lat() * 10000000.;
          gps_data.longitude = gps.location.lng() * 10000000.;
        } else {
          gps_data.resetLocation();
        }

        if (gps.time.isValid() && gps.date.isValid()) {
          if (xQueueSend(xQueueCombined, &combined, 0)) {
            ++gps_queue_enqueue_count;
          } else {
            ++gps_queue_full_count;
          }
        }
      }
      gps_line_read_count = gps.passedChecksum();
      gps_line_invalid_count = gps.failedChecksum();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void taskPrintStats(void *) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
    if (xWasDelayed == pdFALSE) {
      ESP_LOGW(TAG, "stats task was not delayed, i.e. running too long!");
    }
    stats();
  }
}

esp_err_t queue_setup() {
  xQueueCombined = xQueueCreate(16, sizeof(xQueueItem));
  if (xQueueCombined == 0) {
    ESP_LOGE(TAG, "failed queue setup");
    return ESP_FAIL;
  }

  return ESP_OK;
}


// #define TEST
#ifdef TEST
#include <unity.h>
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(500);
  }
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set(TAG, ESP_LOG_DEBUG);

  UNITY_BEGIN();
  // RUN_TEST(test_convertToDecimalDegrees);
  UNITY_END();
}
#else
void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN, LOW);
  queue_setup();
  xTaskCreatePinnedToCore(taskSendBle, "BLE messages sender", 16384, nullptr, 2, nullptr, 0);  // Core 0 has less other stuff running on it.
  ble_setup();
  canBusSetup();
  xTaskCreatePinnedToCore(taskCanBusLoop, "CAN bus reader", 16384, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskReadGPS, "GPS reader", 16384, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskPrintStats, "Statistics printer", 16384, nullptr, 1, nullptr, 1);
}
#endif

void loop() {
  delay(1000000);
}
