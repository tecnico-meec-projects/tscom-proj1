#include <Arduino.h>
#include <ArduinoBLE.h>

#include "UltrasonicSensor.hpp"
#include "imu.hpp"
#include "BleSensorService.hpp"
#include "Pedometer.hpp"

// Pins
static constexpr uint8_t trigPin = 2;
static constexpr uint8_t echoPin = 3;

UltrasonicSensor echoSensor(trigPin, echoPin);

// IMU instance
IMU imu;

// BLE service
BleSensorService bleService;

// Pedometer (classe com o algoritmo da ADI)
Pedometer pedometer;

// Timers
unsigned long lastPrint      = 0;
unsigned long lastStepUpdate = 0;

// Última aceleração lida (mg)
int16_t lastAx_mg = 0;
int16_t lastAy_mg = 0;
int16_t lastAz_mg = 0;

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

  pedometer.reset();

  lastPrint      = millis();
  lastStepUpdate = millis();
}

void loop()
{
  unsigned long now = millis();

  echoSensor.update();

  // ================= PEDÓMETRO A ~50 Hz (ODR do algoritmo) =================
  if (now - lastStepUpdate >= 20)   // 20 ms ≈ 50 Hz
  {
    lastStepUpdate += 20;

    // Ler acelerómetro
    imu.readAcceleration();

    lastAx_mg = imu.getAccelX_mg();
    lastAy_mg = imu.getAccelY_mg();
    lastAz_mg = imu.getAccelZ_mg();

    // Atualizar o algoritmo de passos
    pedometer.update(lastAx_mg, lastAy_mg, lastAz_mg);
  }

  // ================= DISTÂNCIA ULTRASSÔNICA =================
  float distance = echoSensor.getDistance(); // cm

  // Converter mg → g só para debug / BLE
  float ax_g = lastAx_mg / 1000.0f;
  float ay_g = lastAy_mg / 1000.0f;
  float az_g = lastAz_mg / 1000.0f;

  // ================= BLE UPDATE =================
  // Exemplo de atualização do serviço BLE com passos e distância
  float strideLength   = pedometer.getLastStrideLength();     // m
  float totalDistance  = pedometer.getTotalDistance();        // m
  float avgStepLength  = pedometer.getAverageStepLength();    // m/step
  uint32_t stepCount   = pedometer.getStepCount();

  bleService.update(distance, ax_g, ay_g, az_g);

  // ================= DEBUG SERIAL =================
  if (now - lastPrint > 200)
  {
    Serial.print("Distance (cm): ");
    Serial.println(distance);

    Serial.print("Accel (g):  ");
    Serial.print(ax_g, 3); Serial.print(", ");
    Serial.print(ay_g, 3); Serial.print(", ");
    Serial.println(az_g, 3);

    Serial.print("Steps: ");
    Serial.println(stepCount);

    Serial.print("Last stride length (m): ");
    Serial.println(strideLength, 3);

    Serial.print("Average step length (m): ");
    Serial.println(avgStepLength, 3);

    Serial.print("Total distance (m): ");
    Serial.println(totalDistance, 3);

    Serial.println("---------------------");

    lastPrint = now;
  }
}
