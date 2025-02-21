#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <esp_log.h>
#include "task.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "ble_pipe";

#define SERVICE_UUID "00001ff8-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

bool isBleConnected = false;
uint16_t conn_id = 0;  // Only valid when isBleConnected is true.
BLECharacteristic *cbMainChar = nullptr;
BLERemoteCharacteristic *a;

struct foo {
  uint8_t buf[8];
};

#define notificationType struct foo
#define notificationTypeInit \
  { 0 }
#define BUFF_SIZE 20


// stats
static uint64_t ble_notify_count = 0;
static uint64_t ble_no_tx_buf_evt_count = 0;

void stats() {
  static uint64_t last_qec = 0;
  static uint64_t last_qdc = 0;

  uint64_t diff_qec = ble_notify_count - last_qec;
  uint64_t diff_qdc = ble_no_tx_buf_evt_count - last_qdc;

  last_qec = ble_notify_count;
  last_qdc = ble_no_tx_buf_evt_count;

  Serial.printf("ble_notify_count/s %llu, ", diff_qec);
  Serial.printf("ble_notify_bytes/s %llu, ", diff_qec * BUFF_SIZE);
  Serial.printf("ble_no_tx_buf_evt_count/s %llu, ", diff_qdc);
  // Serial.printf("queue_dequeue_bytes/s %llu, ", diff_qdc * sizeof(notificationType));
  Serial.println("");
}

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
  BLEDevice::setPower(ESP_PWR_LVL_P20);
  BLEDevice::setPower(ESP_PWR_LVL_P20, ESP_BLE_PWR_TYPE_CONN_HDL0);
  ESP_ERROR_CHECK(esp_ble_gap_set_preferred_default_phy(
    ESP_BLE_GAP_PHY_2M_PREF_MASK,
    ESP_BLE_GAP_PHY_2M_PREF_MASK));
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCanbusMainCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x01), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  cbMainChar = pCanbusMainCharacteristic;

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

void taskPrintStats(void *) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    BaseType_t xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
    if (xWasDelayed == pdFALSE) {
      ESP_LOGW(TAG, "sim task was not delayed, i.e. running too long!");
    }
    stats();
  }
}

void taskSendBle(void *) {
  notificationType message = notificationTypeInit;
  for (;;) {
    if (!isBleConnected) {
      vTaskDelay(500);
      continue;
    }
    int free_buff_num = esp_ble_get_cur_sendable_packets_num(conn_id);
    if (free_buff_num == 0) {
      ++ble_no_tx_buf_evt_count;
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }
    for (; free_buff_num > 0; free_buff_num--) {
      uint32_t id = 0x900;
      uint8_t buf[BUFF_SIZE] = { 0 };
      buf[0] = (uint8_t)(id >> 0);
      buf[1] = (uint8_t)(id >> 8);
      buf[2] = (uint8_t)(id >> 16);
      buf[3] = (uint8_t)(id >> 24);
      cbMainChar->setValue(buf, sizeof(buf));
      cbMainChar->notify();
      ++ble_notify_count;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // wait for serial port to connect. Needed for native USB

  esp_log_level_set("*", ESP_LOG_ERROR);
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  ble_setup();
  xTaskCreatePinnedToCore(taskPrintStats, "Statistics printer", 16384, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskSendBle, "BLE messages sender", 16384, nullptr, 2, nullptr, 0);  // Core 0 has less other stuff running on it.
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000000);
}
