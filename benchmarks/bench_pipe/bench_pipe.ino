#include <Arduino.h>

#include <esp_log.h>
#include "task.h"


static const char *TAG = "bench_pipe";

struct foo {
  uint8_t buf[256];
};

#define queueType struct foo
#define queueTypeInit {}
#define queueSize 8

// Results:
//  qz |    ops/s | taskprio 20 | taskprio 1 | taskprio 0 |  iz 16 |  iz 32 |  iz 64 | iz 128 | iz 256 |
// -----------------------------------------------------------------------------------------------------
//   1 |    52994 |
//   2 |   105936 |
//   4 |   617344 |
//   8 |   621006 |      632807 |     632822 |     314995 | 618535 | 600543 | 574197 | 461385 | 347784 |
// 128 |   630990 |
//2^15 |   631060 |


QueueHandle_t xQueue1;

// stats
static uint64_t queue_enqueue_count = 0;
static uint64_t queue_dequeue_count = 0;

void stats() {
  static uint64_t last_qec = 0;
  static uint64_t last_qdc = 0;

  uint64_t diff_qec = queue_enqueue_count - last_qec;
  uint64_t diff_qdc = queue_dequeue_count - last_qdc;

  last_qec = queue_enqueue_count;
  last_qdc = queue_dequeue_count;

  Serial.printf("queue_enqueue_count/s %llu, ", diff_qec);
  Serial.printf("queue_enqueue_bytes/s %llu, ", diff_qec * sizeof(queueType));
  Serial.printf("queue_dequeue_count/s %llu, ", diff_qdc);
  Serial.printf("queue_dequeue_bytes/s %llu, ", diff_qdc * sizeof(queueType));
  Serial.println("");
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

void producer(void *) {
  constexpr int batch_size = 400000;
  queueType item = queueTypeInit;
  for (;;) {
    for (int i = 0; i < batch_size; ++i) {
      if (xQueueSend(xQueue1, &item, pdMS_TO_TICKS(1000))) {
        ++queue_enqueue_count;
      }
    }
    vTaskDelay(1);
  }
}

void consumer(void *) {
  queueType item;
  for (;;) {
    if (xQueueReceive(xQueue1, &item, pdMS_TO_TICKS(1000))) {
      ++queue_dequeue_count;
    }
  }
}

esp_err_t queue_setup() {
  xQueue1 = xQueueCreate(queueSize, sizeof(queueType));
  if (xQueue1 == 0) {
    ESP_LOGE(TAG, "failed queue setup");
    return ESP_FAIL;
  }

  return ESP_OK;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // wait for serial port to connect. Needed for native USB

  esp_log_level_set("*", ESP_LOG_ERROR);
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  pinMode(LED_BUILTIN, OUTPUT);

  ESP_ERROR_CHECK(queue_setup());

  xTaskCreatePinnedToCore(producer, "producer", 16384, nullptr, 2, nullptr, 0);  // Core 0 has less other stuff running on it.
  xTaskCreatePinnedToCore(consumer, "consumer", 16384, nullptr, 2, nullptr, 1);

  xTaskCreatePinnedToCore(taskPrintStats, "Statistics printer", 16384, nullptr, 20, nullptr, 1);
}

void loop() {
  delay(10000000);
}
