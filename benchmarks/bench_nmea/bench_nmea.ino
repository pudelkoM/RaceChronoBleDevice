#include <Arduino.h>
#include <esp_log.h>
#include "task.h"

static const char* TAG = "bench_nmea";

typedef int32_t (*ConvertFunc)(const String&, const String&);

// Converts latitude and longitude from "[d]ddmm.mmmm" NMEA format to decimal degrees.
static int32_t convertToDecimalDegrees(const String& val, const String& dir) {
  int dotIndex = val.indexOf('.');
  int degrees = val.substring(0, dotIndex - 2).toInt();
  float minutes = val.substring(dotIndex - 2).toFloat();
  float decimalDegrees = degrees + (minutes / 60.);
  if (dir == "S" || dir == "W") {
    decimalDegrees = -decimalDegrees;
  }
  return decimalDegrees * 10000000.;
}

static int32_t convertToDecimalDegreesRound(const String& val, const String& dir) {
  int dotIndex = val.indexOf('.');
  int degrees = val.substring(0, dotIndex - 2).toInt();
  float minutes = val.substring(dotIndex - 2).toFloat();
  float decimalDegrees = degrees + (minutes / 60.);
  if (dir == "S" || dir == "W") {
    decimalDegrees = -decimalDegrees;
  }
  return roundf(decimalDegrees * 10000000.);
}

static int32_t convertToDecimalDegreesDouble(const String& val, const String& dir) {
  int dotIndex = val.indexOf('.');
  int degrees = val.substring(0, dotIndex - 2).toInt();
  double minutes = val.substring(dotIndex - 2).toDouble();
  double decimalDegrees = degrees + (minutes / 60.);
  if (dir == "S" || dir == "W") {
    decimalDegrees = -decimalDegrees;
  }
  return decimalDegrees * 10000000.;
}

static int32_t convertToDecimalDegreesTinygps(const String& val, const String& dir) {
  const char* term = val.c_str();
  uint32_t leftOfDecimal = (uint32_t)atol(term);
  uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
  uint32_t multiplier = 10000000UL;
  uint32_t tenMillionthsOfMinutes = minutes * multiplier;

  int32_t deg = (int16_t)(leftOfDecimal / 100);

  while (isdigit(*term)) {
    ++term;
  }

  if (*term == '.') {
    while (isdigit(*++term)) {
      multiplier /= 10;
      tenMillionthsOfMinutes += (*term - '0') * multiplier;
    }
  }

  uint32_t billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
  int32_t ret = deg * 10000000UL + billionths / 100;
  if (dir == "S" || dir == "W") {
    ret = -ret;
  }

  return ret;
}

int32_t convertToDecimalDegreesErik(const String& val, const String& _dir) {
  const char* s = val.c_str();

  int32_t degs = *s++ - '0';
  if (degs > 0) {
    degs *= 10;
  }
  degs += *s++ - '0';

  degs *= 10;
  degs += *s++ - '0';

  degs *= 10;  // we've multiplied degrees by 10 so far
  degs += *s++ - '0';
  degs *= 10;  // 100
  degs += *s++ - '0';

  if (*s++ != '.') {
    return -1;  // invalid value, use as error flag
  }

  // four more digits of minute precision
  degs *= 10;  // 1,000
  degs += *s++ - '0';
  degs *= 10;  // 10,000
  degs += *s++ - '0';
  degs *= 10;  // 100,000
  degs += *s++ - '0';

  degs *= 10;          // 1,000,000
  degs += *s++ - '0';  //

  // if (*s++ != ',') {
  //   return -1;
  // }
  // char dir = *s++;
  char dir = _dir.charAt(0);

  // E: 01000101, positive
  // N: 01001110, positive
  // W: 01010111, negative
  // S: 01010011, negative

  // just check for the fifth bit
  int neg = (dir & (1 << 4)) >> 3;  // 0 or 2 for positive or negative
  neg = 1 - neg;                    // 1-0 -> 1 for positive, 1-2 -> -1 for negative
  // this gets rid of a branch which is probably pointless,  but it's nicer than checking for multiple different directions
  return degs * neg * 10;
}


// https://arduino.stackexchange.com/a/94836
static void benchmark_convertFunction(ConvertFunc func, const char* funcName);

static void benchmark_convertFunction(ConvertFunc func, const char* funcName) {
  const int NUM_ITERATIONS = 10000;
  String val = "12447.0949";
  String dir = "S";
  int32_t ret;
  unsigned long start, end;
  float totalTime = 0;

  start = micros();
  for (int i = 0; i < NUM_ITERATIONS; i++) {
    ret = func(val, dir);
  }
  end = micros();
  totalTime = (end - start);

  Serial.print("Average time for ");
  Serial.print(funcName);
  Serial.print(": ");
  Serial.print(totalTime / NUM_ITERATIONS);
  Serial.print(" microseconds.");
  Serial.printf(" Result: %i\n", ret);
}

void taskBench(void*) {
  benchmark_convertFunction(convertToDecimalDegrees, "convertToDecimalDegrees");
  benchmark_convertFunction(convertToDecimalDegreesRound, "convertToDecimalDegreesRound");
  benchmark_convertFunction(convertToDecimalDegreesDouble, "convertToDecimalDegreesDouble");
  benchmark_convertFunction(convertToDecimalDegreesTinygps, "convertToDecimalDegreesTinygps");
  benchmark_convertFunction(convertToDecimalDegreesErik, "convertToDecimalDegreesErik");

  vTaskDelete(nullptr);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(500); }  // wait for serial port to connect. Needed for native USB

  esp_log_level_set("*", ESP_LOG_ERROR);
  esp_log_level_set(TAG, ESP_LOG_DEBUG);
  xTaskCreatePinnedToCore(taskBench, "NMEA benchmark", 16384, nullptr, 2, nullptr, 1);
}

void loop() {
  delay(10000000);
}
