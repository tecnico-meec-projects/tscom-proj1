#include <Arduino.h>
#include <ArduinoBLE.h>

#include "UltrasonicSensor.hpp"
#include "imu.hpp"
#include "BleSensorService.hpp"
#include "Pedometer.hpp"
#include "InertialSpeedEstimator.hpp"
#include "StepSizeEstimator.hpp"   

// Pins
static constexpr uint8_t trigPin = 2;
static constexpr uint8_t echoPin = 3;

UltrasonicSensor echoSensor(trigPin, echoPin);

// IMU instance
IMU imu;

// BLE service
BleSensorService bleService;

// Pedometer
Pedometer pedometer;


InertialSpeedEstimator speedEstimator(0.02f);


StepSizeEstimator stepSizeEstimator;  

// Timers
unsigned long lastPrint      = 0;
unsigned long lastStepUpdate = 0;


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


  Serial.println("Calibrating inertial speed (keep device still)...");
  speedEstimator.startCalibration();
  unsigned long calibStart = millis();
  while (millis() - calibStart < 2000)   
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
  stepSizeEstimator.reset();
  lastPrint      = millis();
  lastStepUpdate = millis();
}

void loop()
{
  unsigned long now = millis();

  echoSensor.update();


  if (now - lastStepUpdate >= 20)   // 20 ms ≈ 50 Hz
  {
    lastStepUpdate += 20;


    imu.readAcceleration();

    lastAx_mg = imu.getAccelX_mg();
    lastAy_mg = imu.getAccelY_mg();
    lastAz_mg = imu.getAccelZ_mg();


    pedometer.update(lastAx_mg, lastAy_mg, lastAz_mg);


    speedEstimator.update(lastAx_mg, lastAy_mg, lastAz_mg);
    

    float currentSpeed = speedEstimator.getSpeedMps();
    uint32_t stepCount = pedometer.getStepCount();
    stepSizeEstimator.update(currentSpeed, stepCount);
  }


  float distance = echoSensor.getDistance(); // cm


  float ax_g = lastAx_mg / 1000.0f;
  float ay_g = lastAy_mg / 1000.0f;
  float az_g = lastAz_mg / 1000.0f;

  // Get step metrics
  uint32_t stepCount = pedometer.getStepCount();
  float speed_mps = speedEstimator.getSpeedMps();
  float stepSize_cm = stepSizeEstimator.getStepSizeCm();
  float cadence = stepSizeEstimator.getCadence();
  
  // Update BLE with all data
  bleService.update(distance, ax_g, ay_g, az_g, stepCount, speed_mps, stepSize_cm, cadence);


  if (now - lastPrint > 100)
  {
    float speed_mps = speedEstimator.getSpeedMps();
    float speed_kmh = speedEstimator.getSpeedKmh();
    uint32_t stepCount = pedometer.getStepCount();
    float stepSize_cm = stepSizeEstimator.getStepSizeCm();
    float avgStepSize_cm = stepSizeEstimator.getAverageStepSizeMeters() * 100.0f;
    float cadence = stepSizeEstimator.getCadence();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    Serial.print("Accel (g):  ");
    Serial.print(ax_g, 3); Serial.print(", ");
    Serial.print(ay_g, 3); Serial.print(", ");
    Serial.println(az_g, 3);

    Serial.print("Steps: ");
    Serial.println(stepCount);

    Serial.print("Speed: ");
    Serial.print(speed_mps, 2);
    Serial.print(" m/s  (");
    Serial.print(speed_kmh, 2);
    Serial.println(" km/h)");


    Serial.print("Step Size: ");
    Serial.print(stepSize_cm, 1);
    Serial.print(" cm  (Avg: ");
    Serial.print(avgStepSize_cm, 1);
    Serial.println(" cm)");
    
    Serial.print("Cadence: ");
    Serial.print(cadence, 2);
    Serial.println(" steps/sec");

    Serial.println("---------------------");

    lastPrint = now;
  }
}