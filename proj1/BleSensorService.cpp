
#include "BleSensorService.hpp"

BleSensorService::BleSensorService()
  : sensorService("19B10000-E8F2-537E-4F6C-D104768A1214"),
    
    distanceChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                 BLERead | BLENotify, 16),
    
    stepCountChar("19B10002-E8F2-537E-4F6C-D104768A1214",
                  BLERead | BLENotify, 16),
    
    stepLengthChar("19B10003-E8F2-537E-4F6C-D104768A1214",
                   BLERead | BLENotify, 16),
    
    statusChar("19B10004-E8F2-537E-4F6C-D104768A1214",
               BLERead | BLENotify, 32)
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

  // Add all characteristics to the service
  sensorService.addCharacteristic(distanceChar);
  sensorService.addCharacteristic(stepCountChar);
  sensorService.addCharacteristic(stepLengthChar);
  sensorService.addCharacteristic(statusChar);

  // Initialize values
  distanceChar.writeValue("0.0");
  stepCountChar.writeValue("0");
  stepLengthChar.writeValue("0.0");
  statusChar.writeValue("NORMAL");

  // Add service and start advertising
  BLE.addService(sensorService);
  BLE.setAdvertisedService(sensorService);
  BLE.advertise();

  Serial.println("BLE service started with characteristics:");
  Serial.println("  - Distance (cm)");
  Serial.println("  - Step Count");
  Serial.println("  - Step Length (m)");
  Serial.println("  - Status (shake/wall warnings)");
  
  return true;
}

void BleSensorService::update(float distance_cm, uint32_t stepCount, float stepLength_m,
                              bool shakeDetected, bool wallWarning)
{
  BLE.poll();

  unsigned long now = millis();
  if (now - lastBleUpdate < bleUpdateInterval)
    return;

  lastBleUpdate = now;

  // Convert numeric values to strings
  String distStr   = String(distance_cm, 2);   // 2 decimal places
  String countStr  = String(stepCount);         // Integer
  String lengthStr = String(stepLength_m, 3);  // 3 decimal places

  // Build status string
  String statusStr = "";
  if (shakeDetected && wallWarning)
    statusStr = "SHAKE+WALL";
  else if (shakeDetected)
    statusStr = "SHAKING";
  else if (wallWarning)
    statusStr = "WALL_NEAR";
  else
    statusStr = "NORMAL";

  // Update all characteristics
  distanceChar.writeValue(distStr);
  stepCountChar.writeValue(countStr);
  stepLengthChar.writeValue(lengthStr);
  statusChar.writeValue(statusStr);
}