#include "BleSensorService.hpp"

BleSensorService::BleSensorService()
  : serviceDistAx("19B10000-E8F2-537E-4F6C-D104768A1214"),
    serviceAyAz ("19B20000-E8F2-537E-4F6C-D104768A1214"),

    // Service 1: distance + Ax  
    distanceChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    accelXChar ("19B10002-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),

    // Service 2: Ay + Az  
    accelYChar ("19B20001-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    accelZChar ("19B20002-E8F2-537E-4F6C-D104768A1214",
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


  serviceDistAx.addCharacteristic(distanceChar);
  serviceDistAx.addCharacteristic(accelXChar);


  serviceAyAz.addCharacteristic(accelYChar);
  serviceAyAz.addCharacteristic(accelZChar);


  distanceChar.writeValue("0.0");
  accelXChar.writeValue("0.0");
  accelYChar.writeValue("0.0");
  accelZChar.writeValue("0.0");


  BLE.addService(serviceDistAx);
  BLE.addService(serviceAyAz);


  BLE.setAdvertisedService(serviceDistAx);
  BLE.advertise();

  Serial.println("BLE services started (Dist+Ax) e (Ay+Az) [STRING]");
  return true;
}

void BleSensorService::update(float distance_cm, float ax_g, float ay_g, float az_g)
{
  BLE.poll();

  unsigned long now = millis();
  if (now - lastBleUpdate < bleUpdateInterval)
    return;

  lastBleUpdate = now;


  String dStr  = String(distance_cm, 2);
  String axStr = String(ax_g,        3);
  String ayStr = String(ay_g,        3);
  String azStr = String(az_g,        3);

  // Service 1
  distanceChar.writeValue(dStr);
  accelXChar.writeValue(axStr);

  // Service 2
  accelYChar.writeValue(ayStr);
  accelZChar.writeValue(azStr);
}