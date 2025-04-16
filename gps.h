#ifndef GPSHEADER
#define GPSHEADER

#include <Arduino.h>
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

  // GPRMC
  uint16_t speedOverGround;   // km/h * 100
  uint16_t courseOverGround;  // degrees * 100

  // GPGSA
  uint8_t hdop;  // hdop * 10
  uint8_t vdop;  // vdop * 10

  GpsData()
    : gpsSyncBits(), dateAndHour(), hours(), minutes(), seconds(), milliseconds(),
      latitude(0x7FFFFFFF), longitude(0x7FFFFFFF), fixQuality(0x00), numberOfSatellites(0x3F),
      altitude(0xFFFF), speedOverGround(0xFFFF), courseOverGround(0xFFFF), hdop(0xFF), vdop(0xFF) {}

  void resetLocation() {
    latitude = 0x7FFFFFFF;
    longitude = 0x7FFFFFFF;
    fixQuality = 0x00;
    numberOfSatellites = 0x3F;
    altitude = 0xFFFF;
    speedOverGround = 0xFFFF;
    courseOverGround = 0xFFFF;
    hdop = vdop = 0xFF;
  }
};

#endif
