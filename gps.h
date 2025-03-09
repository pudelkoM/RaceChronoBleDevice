#ifndef GPSHEADER
#define GPSHEADER

#include <Arduino.h>
#include <unity.h>
#include <cmath>

struct GpsData {
  // RaceChrono specific data.
  uint8_t gpsSyncBits;
  uint32_t dateAndHour;

  uint32_t hours;
  uint32_t minutes;
  uint32_t seconds;
  uint32_t milliseconds;

  // GPGGA
  int32_t latitude;   // decimal degrees * 10'000'000
  int32_t longitude;  // decimal degrees * 10'000'000
  uint8_t fixQuality;
  uint8_t numberOfSatellites;
  uint16_t altitude;
  // String altitudeUnits;
  // String geoidalSeparation;
  // String geoidalSeparationUnits;
  // String ageOfDifferentialGPSData;
  // String differentialReferenceStationID;

  // GPRMC
  uint16_t speedOverGround;   // km/h * 100
  uint16_t courseOverGround;  // degrees * 100

  // GPGSA
  uint8_t hdop;  // hdop * 10
  uint8_t vdop;  // vdop * 10
};

// Converts latitude and longitude from "[d]ddmm.mmmm" NMEA format to decimal degrees.
static int32_t convertToDecimalDegrees(String val, String dir) {
  int dotIndex = val.indexOf('.');
  int degrees = val.substring(0, dotIndex - 2).toInt();
  float minutes = val.substring(dotIndex - 2).toFloat();
  float decimalDegrees = degrees + (minutes / 60.);
  if (dir == "S" || dir == "W") {
    decimalDegrees = -decimalDegrees;
  }
  return roundf(decimalDegrees * 10000000.);
}

static int32_t convertToDecimalDegreesRound(String val, String dir) {
  int dotIndex = val.indexOf('.');
  int degrees = val.substring(0, dotIndex - 2).toInt();
  float minutes = val.substring(dotIndex - 2).toFloat();
  float decimalDegrees = degrees + (minutes / 60.);
  if (dir == "S" || dir == "W") {
    decimalDegrees = -decimalDegrees;
  }
  return decimalDegrees * 10000000.;
}

static int32_t convertToDecimalDegreesDouble(String val, String dir) {
  int dotIndex = val.indexOf('.');
  int degrees = val.substring(0, dotIndex - 2).toInt();
  double minutes = val.substring(dotIndex - 2).toDouble();
  double decimalDegrees = degrees + (minutes / 60.);
  if (dir == "S" || dir == "W") {
    decimalDegrees = -decimalDegrees;
  }
  return decimalDegrees * 10000000.;
}

static int32_t convertToDecimalDegreesTinygps(String val, String dir) {
  const char *term = val.c_str();
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

static void test_convertToDecimalDegrees() {
  struct gps_data {
    String val;
    String dir;
    int32_t expected;
  };
  struct gps_data test_data[] = {
    { "3724.0313", "S", -374005216 },
    { "3724.0313", "N", 374005216 },
    { "3724.0314", "N", 374005216 },
    { "3724.0315", "N", 374005216 },
    { "3724.0316", "N", 374005216 },
    { "3724.0317", "N", 374005216 },
    { "3724.0318", "N", 374005216 },
    { "03724.0313", "N", 374005216 },
    { "12204.5543", "W", -1220759040 },
    { "12204.5543", "E", 1220759040 },
    { "12204.5544", "E", 1220759040 },
    { "12204.5545", "E", 1220759040 },
    { "12204.5546", "E", 1220759040 },
    { "12204.5547", "E", 1220759040 },
    { "0.0", "E", 0 },
    { "3724.0661", "N", 374005216 },
    { "12204.6332", "W", 374005216 },
  };
  for (int i = 0; i < sizeof(test_data) / sizeof(test_data[0]); i++) {

    int32_t result1 = convertToDecimalDegrees(test_data[i].val, test_data[i].dir);
    int32_t result2 = convertToDecimalDegreesDouble(test_data[i].val, test_data[i].dir);
    int32_t result3 = convertToDecimalDegreesTinygps(test_data[i].val, test_data[i].dir);
    int32_t result4 = convertToDecimalDegreesRound(test_data[i].val, test_data[i].dir);

    Serial.print("Test case ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(test_data[i].val);
    Serial.print(", ");
    Serial.print(test_data[i].dir);
    Serial.print(" -> ");
    Serial.print("convertToDecimalDegreesFloat: ");
    Serial.print(result1);
    Serial.print(", convertToDecimalDegreesDouble: ");
    Serial.print(result2);
    Serial.print(", convertToDecimalDegreesTinygps: ");
    Serial.print(result3);
    Serial.print(", convertToDecimalDegreesRound: ");
    Serial.println(result4);
  }
}

typedef int32_t (*ConvertFunc)(String, String);

void benchmark_convertFunction(ConvertFunc func, const char *funcName) {
  const int NUM_ITERATIONS = 10000;
  String val = "12447.0949";
  String dir = "S";
  unsigned long start, end;
  float totalTime = 0;

  start = micros();
  for (int i = 0; i < NUM_ITERATIONS; i++) {
    func(val, dir);
  }
  end = micros();
  totalTime = (end - start);

  Serial.print("Average time for ");
  Serial.print(funcName);
  Serial.print(": ");
  Serial.print(totalTime / NUM_ITERATIONS);
  Serial.println(" microseconds");
}

static bool parseGPGGA(const String &nmea, struct GpsData &data) {
  if (!nmea.startsWith("$GPGGA")) {
    return false;  // Invalid message
  }

  // Skip the message ID
  int startIndex = 0;
  int endIndex = nmea.indexOf(',');

  // Time
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  String time = nmea.substring(startIndex, endIndex);
  data.hours = time.substring(0, 2).toInt();
  data.minutes = time.substring(2, 4).toInt();
  data.seconds = time.substring(4, 6).toInt();
  data.milliseconds = time.substring(7).toInt();

  // Latitude
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  String latitude = nmea.substring(startIndex, endIndex);

  // Latitude Direction
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  String latitudeDirection = nmea.substring(startIndex, endIndex);

  data.latitude = convertToDecimalDegreesTinygps(latitude, latitudeDirection);

  // Longitude
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  String longitude = nmea.substring(startIndex, endIndex);

  // Longitude Direction
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  String longitudeDirection = nmea.substring(startIndex, endIndex);

  data.longitude = convertToDecimalDegreesTinygps(longitude, longitudeDirection);

  // Fix Quality
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  data.fixQuality = nmea.substring(startIndex, endIndex).toInt();

  // Number of Satellites
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  data.numberOfSatellites = nmea.substring(startIndex, endIndex).toInt();

  // Horizontal Dilution
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.horizontalDilution = nmea.substring(startIndex, endIndex);

  // Altitude
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  String altitude = nmea.substring(startIndex, endIndex);
  data.altitude = uint16_t((altitude.toFloat() + 500) * 10) & 0x7FFF;

  // Altitude Units
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.altitudeUnits = nmea.substring(startIndex, endIndex);

  // Geoidal Separation
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.geoidalSeparation = nmea.substring(startIndex, endIndex);

  // Geoidal Separation Units
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.geoidalSeparationUnits = nmea.substring(startIndex, endIndex);

  // Age of Differential GPS Data
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.ageOfDifferentialGPSData = nmea.substring(startIndex, endIndex);

  // Differential Reference Station ID
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf('*', startIndex);
  // data.differentialReferenceStationID = nmea.substring(startIndex, endIndex);

  return true;
}

static bool parseGPRMC(const String &nmea, struct GpsData &data) {
  if (!nmea.startsWith("$GPRMC")) {
    return false;  // Invalid message
  }
  // Skip the message ID
  int startIndex = 0;
  int endIndex = nmea.indexOf(',');

  // Time
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  String time = nmea.substring(startIndex, endIndex);
  int hour = time.substring(0, 2).toInt();
  int minute = time.substring(2, 4).toInt();
  int second = time.substring(4, 6).toInt();
  int millisecond = time.substring(7).toInt();

  // Status
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.status = nmea.substring(startIndex, endIndex);

  // Latitude
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.latitude = nmea.substring(startIndex, endIndex);

  // Latitude Direction
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.latitudeDirection = nmea.substring(startIndex, endIndex);

  // Longitude
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.longitude = nmea.substring(startIndex, endIndex);

  // Longitude Direction
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.longitudeDirection = nmea.substring(startIndex, endIndex);

  // Speed Over Ground
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  data.speedOverGround = uint16_t(nmea.substring(startIndex, endIndex).toFloat() * 1.852 * 100.) & 0x7FFF;

  // Course Over Ground
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  data.courseOverGround = uint16_t(nmea.substring(startIndex, endIndex).toFloat() * 100.) & 0x7FFF;

  // Date
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  String date = nmea.substring(startIndex, endIndex);
  int day = date.substring(0, 2).toInt();
  int month = date.substring(2, 4).toInt();
  int year = date.substring(4, 6).toInt();
  data.dateAndHour = (year * 8928) + ((month - 1) * 744) + ((day - 1) * 24) + hour;
  static uint32_t last_date_and_hour = data.dateAndHour;
  if (data.dateAndHour != last_date_and_hour) {
    last_date_and_hour = data.dateAndHour;
    ++data.gpsSyncBits;
  }

  // Mode Indicator
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.modeIndicator = nmea.substring(startIndex, endIndex);

  return true;
}

static bool parseGPGSA(const String &nmea, struct GpsData &data) {
  if (!nmea.startsWith("$GPGSA")) {
    return false;  // Invalid message
  }

  // Skip the message ID
  int startIndex = 0;
  int endIndex = nmea.indexOf(',');

  // Mode
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.mode = nmea.substring(startIndex, endIndex);

  // Fix Mode
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.fixMode = nmea.substring(startIndex, endIndex);

  // Satellites (12)
  for (int i = 0; i < 12; ++i) {
    startIndex = endIndex + 1;
    endIndex = nmea.indexOf(',', startIndex);
    // data.prns = nmea.substring(startIndex, endIndex);
  }

  // PDOP
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  // data.pdop = nmea.substring(startIndex, endIndex);

  // HDOP
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf(',', startIndex);
  data.hdop = uint8_t(nmea.substring(startIndex, endIndex).toFloat() * 10.) & 0x7F;

  // VDOP
  startIndex = endIndex + 1;
  endIndex = nmea.indexOf('*', startIndex);
  data.vdop = uint8_t(nmea.substring(startIndex, endIndex).toFloat() * 10.) & 0x7F;

  return true;
}

static bool parseNMEA(const String &nmea, struct GpsData &data) {
  if (nmea.startsWith("$GPGGA")) {
    return parseGPGGA(nmea, data);
  } else if (nmea.startsWith("$GPGSA")) {
    return parseGPGSA(nmea, data);
  } else if (nmea.startsWith("$GPRMC")) {
    return parseGPRMC(nmea, data);
  } else if (nmea.startsWith("$GPGLL")) {
    return true;
  } else if (nmea.startsWith("$GPGSV")) {
    return true;
  } else if (nmea.startsWith("$GPVTG")) {
    return true;
  } else if (nmea.startsWith("$GPZDA")) {
    return true;
  }

  return false;
}

#endif
