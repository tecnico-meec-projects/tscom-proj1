#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>

class BleSensorService
{
public:
  BleSensorService();

  bool begin(const char* deviceName);


  void update(float distance_cm, float ax_g, float ay_g, float az_g, 
              uint32_t steps, float speed_mps, float stepSize_cm, float cadence);

private:
  // Service 1: distance + Ax
  BLEService serviceDistAx;

  // Service 2: Ay + Az
  BLEService serviceAyAz;
  
  // Service 3: Step metrics (steps, speed, step size, cadence)
  BLEService serviceStepMetrics;

  // Service 1 characteristics
  BLEStringCharacteristic distanceChar; 
  BLEStringCharacteristic accelXChar;   
  
  // Service 2 characteristics
  BLEStringCharacteristic accelYChar;   
  BLEStringCharacteristic accelZChar;   
  
  // Service 3 characteristics
  BLEStringCharacteristic stepsChar;           // step count as string
  BLEStringCharacteristic speedChar;           // speed in m/s
  BLEStringCharacteristic stepSizeChar;        // step size in cm
  BLEStringCharacteristic cadenceChar;         // cadence in steps/sec

  unsigned long lastBleUpdate = 0;
  static constexpr unsigned long bleUpdateInterval = 100; // ms
};