#include <Arduino.h>
#include <driver/twai.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "e85.h"
// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include <esp_log.h>
#include "esp_gatt_common_api.h"

static const char *TAG = "racechrono_canbus_ble";

#define CAN_POLLING_RATE_MS 1
#define SERVICE_UUID "00001ff8-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


bool canBusAllowUnknownPackets = false;
bool isCanBusConnected = false;
bool isBleConnected = false;
uint16_t conn_id = 0;  // Only valid when isBleConnected is true.
BLECharacteristic *cbMainChar = nullptr;
QueueHandle_t xQueue1;


static bool canPidAllowed(uint32_t pid) {
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

// Stats and counters
static uint64_t ble_notify_count = 0;
static uint64_t ble_no_tx_buf_evt_count = 0;
static uint64_t can_rx_count = 0;
static uint64_t can_not_interested_count = 0;
static uint64_t can_queue_enqueue_count = 0;
static uint64_t can_queue_full_count = 0;

void stats() {
  static uint64_t last_ble_notify_count = 0;
  static uint64_t last_ble_no_tx_buf_evt_count = 0;
  static uint64_t last_can_rx_count = 0;
  static uint64_t last_can_not_interested_count = 0;
  static uint64_t last_can_queue_enqueue_count = 0;
  static uint64_t last_can_queue_full_count = 0;

  uint64_t diff_ble_notify_count = ble_notify_count - last_ble_notify_count;
  uint64_t diff_ble_no_tx_buf_evt_count = ble_no_tx_buf_evt_count - last_ble_no_tx_buf_evt_count;
  uint64_t diff_can_rx_count = can_rx_count - last_can_rx_count;
  uint64_t diff_can_not_interested_count = can_not_interested_count - last_can_not_interested_count;
  uint64_t diff_can_queue_enqueue_count = can_queue_enqueue_count - last_can_queue_enqueue_count;
  uint64_t diff_can_queue_full_count = can_queue_full_count - last_can_queue_full_count;

  last_ble_notify_count = ble_notify_count;
  last_ble_no_tx_buf_evt_count = ble_no_tx_buf_evt_count;
  last_can_rx_count = can_rx_count;
  last_can_not_interested_count = can_not_interested_count;
  last_can_queue_enqueue_count = can_queue_enqueue_count;
  last_can_queue_full_count = can_queue_full_count;

  Serial.printf("ble_notify_count/s %llu, ", diff_ble_notify_count);
  Serial.printf("ble_notify_bytes/s %llu, ", diff_ble_notify_count * sizeof(twai_message_t));
  Serial.printf("ble_no_tx_buf_evt_count/s %llu, ", diff_ble_no_tx_buf_evt_count);
  Serial.printf("can_rx_count/s %llu, ", diff_can_rx_count);
  Serial.printf("can_not_interested_count/s %llu, ", diff_can_not_interested_count);
  Serial.printf("can_queue_enqueue_count/s %llu, ", diff_can_queue_enqueue_count);
  Serial.printf("can_queue_full_count/s %llu, ", diff_can_queue_full_count);
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
  void onConnect(BLEServer *pServer) {
    Serial.println("Device connected!");
    ESP_LOGI(TAG, "Device connected!");
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

void canBusSetup() {
  // CAN1 setup.
  Serial.println("Initializing builtin CAN peripheral");
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN1_TX, (gpio_num_t)CAN1_RX, TWAI_MODE_LISTEN_ONLY /*TWAI_MODE_NORMAL*/);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = {
    .acceptance_code = ((0x0100 << 3) << 16) | (0x0400 << 3),
    .acceptance_mask = 0xF7FFDFFF,
    .single_filter = false,
  };

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

void canBusLoop() {
  // Manage CAN-Bus connection
  if (!isCanBusConnected && isBleConnected) {
    // Connect to CAN-Bus
    Serial.println("Connecting CAN-Bus...");
    if (twai_start() == ESP_OK) {
      isCanBusConnected = true;
      Serial.println("CAN1 interface started");
    } else {
      Serial.println("Failed to start CAN1");
      delay(3000);
      return;
    }
  } else if (isCanBusConnected && !isBleConnected) {
    // Disconnect from CAN-Bus
    twai_stop();
    isCanBusConnected = false;
    Serial.println("Stopped CAN1");
  }

  // Handle CAN-Bus data
  if (!isCanBusConnected) {  // TODO: use driver status as flag
    vTaskDelay(pdMS_TO_TICKS(500));
    return;
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

  twai_message_t message;
  while (twai_receive(&message, pdMS_TO_TICKS(CAN_POLLING_RATE_MS)) == ESP_OK) {
    ++can_rx_count;
    if (message.rtr) {
      ++can_not_interested_count;
      continue;
    }
    if (!canPidAllowed(message.identifier)) {
      ++can_not_interested_count;
      continue;
    }
    if (xQueueSend(xQueue1, &message, 0)) {
      ++can_queue_enqueue_count;
    } else {
      ++can_queue_full_count;
    }
  }
}

void taskSendBle(void *) {
  twai_message_t message;
  for (;;) {
    if (xQueueReceive(xQueue1, &message, pdMS_TO_TICKS(1000))) {
      sendCanMsgBle(message.identifier, message.data, message.data_length_code);
    }
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
  xQueue1 = xQueueCreate(8, sizeof(twai_message_t));
  if (xQueue1 == 0) {
    ESP_LOGE(TAG, "failed queue setup");
    return ESP_FAIL;
  }

  return ESP_OK;
}


void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_ERROR);
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  pinMode(LED_BUILTIN, OUTPUT);
  queue_setup();
  xTaskCreatePinnedToCore(taskSendBle, "BLE messages sender", 16384, nullptr, 2, nullptr, 0);  // Core 0 has less other stuff running on it.
  ble_setup();
  canBusSetup();
  xTaskCreatePinnedToCore(taskCanBusLoop, "CAN bus reader", 16384, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(taskPrintStats, "Statistics printer", 16384, nullptr, 1, nullptr, 1);
}

void taskCanBusLoop(void *) {
  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    canBusLoop();
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void loop() {
  delay(1000000);
}
