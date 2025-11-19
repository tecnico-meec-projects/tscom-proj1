#include <Arduino.h>
#include <ArduinoBLE.h>

#include "UltrasonicSensor.hpp"
#include "imu.hpp"
#include "BleSensorService.hpp"
#include "Pedometer.hpp"
#include "InertialSpeedEstimator.hpp"   // <--- NOVO

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

// Estimador de velocidade por integração da aceleração
// 0.02 s = 50 Hz (igual ao teu loop de pedómetro)
InertialSpeedEstimator speedEstimator(0.02f);

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

  // --- CALIBRAÇÃO DO ESTIMADOR INERCIAL ---
  Serial.println("Calibrating inertial speed (keep device still)...");
  speedEstimator.startCalibration();
  unsigned long calibStart = millis();
  while (millis() - calibStart < 2000)   // ~2 s parado
  {
    imu.readAcceleration();
    speedEstimator.addCalibrationSample(
      imu.getAccelX_mg(),
      imu.getAccelY_mg(),
      imu.getAccelZ_mg()
    );
    delay(10);
  }
  speedEstimator.finishCalibration();
  Serial.print("Calibration done. g_est = ");
  Serial.println(speedEstimator.getEstimatedG(), 4);

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

  // ================= PEDÓMETRO + VELOCIDADE A ~50 Hz =================
  if (now - lastStepUpdate >= 20)   // 20 ms ≈ 50 Hz
  {
    lastStepUpdate += 20;

    // Ler acelerómetro
    imu.readAcceleration();

    lastAx_mg = imu.getAccelX_mg();
    lastAy_mg = imu.getAccelY_mg();
    lastAz_mg = imu.getAccelZ_mg();

    // Atualizar pedómetro
    pedometer.update(lastAx_mg, lastAy_mg, lastAz_mg);

    // Atualizar velocidade integrada
    speedEstimator.update(lastAx_mg, lastAy_mg, lastAz_mg);
  }

  // ================= DISTÂNCIA + BLE =================
  float distance = echoSensor.getDistance(); // cm

  // Converter mg → g só para debug / BLE
  float ax_g = lastAx_mg / 1000.0f;
  float ay_g = lastAy_mg / 1000.0f;
  float az_g = lastAz_mg / 1000.0f;

  // Continua igual
  bleService.update(distance, ax_g, ay_g, az_g);

  // ================= DEBUG SERIAL =================
  if (now - lastPrint > 100)   // 100 ms → atualização rápida e estável
  {
    float speed_mps = speedEstimator.getSpeedMps();
    float speed_kmh = speedEstimator.getSpeedKmh();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    Serial.print("Accel (g):  ");
    Serial.print(ax_g, 3); Serial.print(", ");
    Serial.print(ay_g, 3); Serial.print(", ");
    Serial.println(az_g, 3);

    Serial.print("Steps: ");
    Serial.println(pedometer.getStepCount());

    Serial.print("Speed: ");
    Serial.print(speed_mps, 2);
    Serial.print(" m/s  (");
    Serial.print(speed_kmh, 2);
    Serial.println(" km/h)");

    Serial.println("---------------------");

    lastPrint = now;
  }
}
