#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>

class BleSensorService
{
public:
  BleSensorService();


  bool begin(const char* deviceName);


  void update(float distance_cm, float ax_g, float ay_g, float az_g);

private:
  // Service 1: distance + Ax
  BLEService serviceDistAx;

  // Service 2: Ay + Az
  BLEService serviceAyAz;


  BLEStringCharacteristic distanceChar; // serviceDistAx
  BLEStringCharacteristic accelXChar;   // serviceDistAx
  BLEStringCharacteristic accelYChar;   // serviceAyAz
  BLEStringCharacteristic accelZChar;   // serviceAyAz

  unsigned long lastBleUpdate = 0;
  static constexpr unsigned long bleUpdateInterval = 100; // ms
};