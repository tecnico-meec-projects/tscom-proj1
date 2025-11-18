#include <Arduino.h>
#include <ArduinoBLE.h>

#include "UltrasonicSensor.hpp"
#include "imu.hpp"
#include "BleSensorService.hpp"

// Pins
static constexpr uint8_t trigPin = 2;
static constexpr uint8_t echoPin = 3;

UltrasonicSensor echoSensor(trigPin, echoPin);

// IMU instance
IMU imu;

// BLE service
BleSensorService bleService;


unsigned long lastPrint = 0;

void setup()
{
  Serial.begin(115200);
  delay(2000);

  echoSensor.begin();

  // IMU
  if (!imu.begin())
  {
    Serial.println("IMU FAILED TO INIT!");
    while (1);
  }
  else
  {
    Serial.println("IMU OK");
  }

  // BLE
  if (!bleService.begin("Nano33BLE-Sensor"))
  {
    Serial.println("BLE FAILED TO INIT!");
    while (1);
  }
  else
  {
    Serial.println("BLE READY, ADVERTISING...");
  }

  lastPrint = millis();
}

void loop()
{

  echoSensor.update();


  imu.readAcceleration();

  float ax = imu.getAccelX_mg() / 1000.0f; // g
  float ay = imu.getAccelY_mg() / 1000.0f; // g
  float az = imu.getAccelZ_mg() / 1000.0f; // g

  float distance = echoSensor.getDistance(); // cm


  bleService.update(distance, ax, ay, az);


  if (millis() - lastPrint > 200)
  {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    Serial.print("Accel (g):  ");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.println(az, 3);

    Serial.println("---------------------");

    lastPrint = millis();
  }
}