// ======================== main.ino ========================
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

// Pedometer 
Pedometer pedometer;

// Timers
unsigned long lastPrint      = 0;
unsigned long lastStepUpdate = 0;


int16_t lastAx_mg = 0;
int16_t lastAy_mg = 0;
int16_t lastAz_mg = 0;


static constexpr float WALL_WARNING_DISTANCE = 30.0f;  
static constexpr float WALL_CRITICAL_DISTANCE = 15.0f; 

// Shake detection variables
uint32_t lastValidStepCount = 0;
unsigned long lastShakeWarning = 0;
static constexpr unsigned long SHAKE_WARNING_INTERVAL = 3000; // Only warn every 3s

// State flags
bool wallWarningActive = false;
bool shakeDetected = false;

void setup()
{
  Serial.begin(115200);
  delay(2000);

  echoSensor.begin();


  if (!imu.begin())
  {
    Serial.println("IMU FAILED TO INIT!");
    while (1);
  }
  else
  {
    Serial.println("IMU OK - I2C communication established");
  }


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


  if (now - lastStepUpdate >= 20)   // 20 ms ≈ 50 Hz (timer-based)
  {
    lastStepUpdate += 20;


    imu.readAcceleration();

    lastAx_mg = imu.getAccelX_mg();
    lastAy_mg = imu.getAccelY_mg();
    lastAz_mg = imu.getAccelZ_mg();


    pedometer.update(lastAx_mg, lastAy_mg, lastAz_mg);
  }

  // ULTRASONIC DISTANCE 
  float distance = echoSensor.getDistance(); // cm

  //  GET PEDOMETER DATA 
  uint32_t stepCount  = pedometer.getStepCount();
  float stepLength    = pedometer.getLastStrideLength();  // m
  float totalDistance = pedometer.getTotalDistance();     // m
  float avgStepLength = pedometer.getAverageStepLength(); // m/step


  // Detect if device is moving but no valid steps are being counted
  // This happens when the pedometer's regulation mode rejects invalid motion
  static unsigned long lastMotionCheck = 0;
  if (now - lastMotionCheck > 500) // Check every 500ms
  {
    lastMotionCheck = now;
    
    // Check if there's significant acceleration but no step increase
    uint32_t accelMagnitude = imu.getAccelMagnitude_mg();
    
    // Significant motion detected (> 1.2g magnitude indicates movement)
    if (accelMagnitude > 1200)
    {
      // But no new steps in last check
      if (stepCount == lastValidStepCount)
      {
        shakeDetected = true;
      }
      else
      {
        shakeDetected = false;
        lastValidStepCount = stepCount;
      }
    }
    else
    {
      // Low acceleration, reset shake flag
      shakeDetected = false;
    }
  }


  if (distance < WALL_WARNING_DISTANCE && distance > 0)
  {
    wallWarningActive = true;
  }
  else
  {
    wallWarningActive = false;
  }


  bleService.update(distance, stepCount, stepLength, shakeDetected, wallWarningActive);


  if (now - lastPrint > 1000)
  {
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║           SENSOR DATA REPORT                   ║");
    Serial.println("╚════════════════════════════════════════════════╝");
    
    // Distance section
    Serial.println("\n┌─ ULTRASONIC SENSOR ─────────────────────────");
    Serial.print("│ Distance: ");
    Serial.print(distance, 2);
    Serial.println(" cm");
    
    // Wall proximity warnings
    if (wallWarningActive)
    {
      if (distance < WALL_CRITICAL_DISTANCE)
      {
        Serial.println("│  STATUS: CRITICAL - WALL TOO CLOSE!");
        Serial.print("│  STOP! Only ");
        Serial.print(distance, 1);
        Serial.println(" cm from wall!");
      }
      else
      {
        Serial.println("│  STATUS: WARNING - Approaching wall");
        Serial.print("│    Distance to wall: ");
        Serial.print(distance, 1);
        Serial.println(" cm");
      }
    }
    else
    {
      Serial.println("│ STATUS: Safe distance");
    }
    Serial.println("└─────────────────────────────────────────────");

    // IMU section
    Serial.println("\n┌─ ACCELEROMETER ─────────────────────────────");
    Serial.print("│ Magnitude: ");
    Serial.print(imu.getAccelMagnitude_mg());
    Serial.println(" mg");
    
    // Shake detection
    if (shakeDetected)
    {
      Serial.println("│  STATUS: SHAKING DETECTED!");
      Serial.println("│  (High motion without valid steps)");
    }
    else
    {
      Serial.println("│  STATUS: Normal motion");
    }
    Serial.println("└─────────────────────────────────────────────");

    // Pedometer section
    Serial.println("\n┌─ PEDOMETER ─────────────────────────────────");
    Serial.print("│ Total Steps: ");
    Serial.println(stepCount);
    
    Serial.print("│ Last Stride: ");
    Serial.print(stepLength, 3);
    Serial.println(" m");
    
    Serial.print("│ Average Step: ");
    Serial.print(avgStepLength, 3);
    Serial.println(" m");
    
    Serial.print("│ Total Distance: ");
    Serial.print(totalDistance, 3);
    Serial.println(" m");
    Serial.println("└─────────────────────────────────────────────");

    // Overall status
    Serial.println("\n┌─ SYSTEM STATUS ─────────────────────────────");
    Serial.print("│ Device State: ");
    if (shakeDetected && wallWarningActive)
      Serial.println("SHAKE + WALL WARNING");
    else if (shakeDetected)
      Serial.println("SHAKING");
    else if (wallWarningActive)
      Serial.println("WALL PROXIMITY");
    else
      Serial.println("NORMAL OPERATION");
    
    Serial.print("│ BLE Status: ");
    if (BLE.connected())
      Serial.println("CONNECTED");
    else
      Serial.println("ADVERTISING");
    Serial.println("└─────────────────────────────────────────────\n");

    lastPrint = now;
  }
}