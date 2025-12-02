// ======================== BleSensorService.hpp ========================
#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>

class BleSensorService
{
public:
  BleSensorService();

  bool begin(const char* deviceName);


  void update(float distance_cm, uint32_t stepCount, float stepLength_m,
              bool shakeDetected, bool wallWarning);

private:
  // Single service for all sensor data
  BLEService sensorService;

  // Characteristics
  BLEStringCharacteristic distanceChar;      // Ultrasonic distance (cm)
  BLEStringCharacteristic stepCountChar;     // Total steps
  BLEStringCharacteristic stepLengthChar;    // Last stride length (m)
  BLEStringCharacteristic statusChar;        // Status flags (shake/wall)

  unsigned long lastBleUpdate = 0;
  static constexpr unsigned long bleUpdateInterval = 100; // ms
};