#include "BleSensorService.hpp"

BleSensorService::BleSensorService()
  : serviceDistAx("19B10000-E8F2-537E-4F6C-D104768A1214"),
    serviceAyAz ("19B20000-E8F2-537E-4F6C-D104768A1214"),
    serviceStepMetrics("19B30000-E8F2-537E-4F6C-D104768A1214"),

    // Service 1: distance + Ax  
    distanceChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    accelXChar ("19B10002-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),

    // Service 2: Ay + Az  
    accelYChar ("19B20001-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    accelZChar ("19B20002-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    
    // Service 3: Step metrics
    stepsChar  ("19B30001-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    speedChar  ("19B30002-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    stepSizeChar("19B30003-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    cadenceChar("19B30004-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16)
{
}

bool BleSensorService::begin(const char* deviceName)
{
  if (!BLE.begin())
  {
    Serial.println("Failed to initialize BLE!");
    return false;
  }

  BLE.setLocalName(deviceName);
  BLE.setDeviceName(deviceName);

  // Service 1: Distance + Ax
  serviceDistAx.addCharacteristic(distanceChar);
  serviceDistAx.addCharacteristic(accelXChar);

  // Service 2: Ay + Az
  serviceAyAz.addCharacteristic(accelYChar);
  serviceAyAz.addCharacteristic(accelZChar);
  
  // Service 3: Step metrics
  serviceStepMetrics.addCharacteristic(stepsChar);
  serviceStepMetrics.addCharacteristic(speedChar);
  serviceStepMetrics.addCharacteristic(stepSizeChar);
  serviceStepMetrics.addCharacteristic(cadenceChar);

  // Initialize values
  distanceChar.writeValue("0.0");
  accelXChar.writeValue("0.0");
  accelYChar.writeValue("0.0");
  accelZChar.writeValue("0.0");
  
  stepsChar.writeValue("0");
  speedChar.writeValue("0.0");
  stepSizeChar.writeValue("0.0");
  cadenceChar.writeValue("0.0");

  // Add all services
  BLE.addService(serviceDistAx);
  BLE.addService(serviceAyAz);
  BLE.addService(serviceStepMetrics);

  // Advertise first service
  BLE.setAdvertisedService(serviceDistAx);
  BLE.advertise();

  Serial.println("BLE services started:");
  Serial.println("  - Service 1: Distance + Ax");
  Serial.println("  - Service 2: Ay + Az");
  Serial.println("  - Service 3: Step Metrics (steps, speed, step size, cadence)");
  
  return true;
}

void BleSensorService::update(float distance_cm, float ax_g, float ay_g, float az_g,
                               uint32_t steps, float speed_mps, float stepSize_cm, float cadence)
{
  BLE.poll();

  unsigned long now = millis();
  if (now - lastBleUpdate < bleUpdateInterval)
    return;

  lastBleUpdate = now;

  // Service 1: Distance + Ax
  String dStr  = String(distance_cm, 2);
  String axStr = String(ax_g,        3);
  distanceChar.writeValue(dStr);
  accelXChar.writeValue(axStr);

  // Service 2: Ay + Az
  String ayStr = String(ay_g, 3);
  String azStr = String(az_g, 3);
  accelYChar.writeValue(ayStr);
  accelZChar.writeValue(azStr);
  
  // Service 3: Step metrics
  String stepsStr = String(steps);
  stepsChar.writeValue(stepsStr);
  
  String speedStr = String(speed_mps, 2);
  speedChar.writeValue(speedStr);
  
  String stepSizeStr = String(stepSize_cm, 1);
  stepSizeChar.writeValue(stepSizeStr);
  
  String cadenceStr = String(cadence, 2);
  cadenceChar.writeValue(cadenceStr);
}