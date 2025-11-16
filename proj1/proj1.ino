#include "UltrasonicSensor.hpp"
#include "imu.hpp"
#include <Arduino.h>

//Pins
static constexpr uint8_t trigPin = 2;
static constexpr uint8_t echoPin = 3;

UltrasonicSensor echoSensor(trigPin, echoPin);

//imu instance
IMU imu;

// Timer
unsigned long lastPrint = 0;

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
    Serial.println("IMU OK");
  }

  lastPrint = millis();
}

void loop()
{
  echoSensor.update();

  // read IMU data
  float ax, ay, az;
  float gx, gy, gz;

  imu.readAcceleration(); // updates internal accelX/Y/Z

  ax = imu.getAccelX_mg() / 1000.0f;
  ay = imu.getAccelY_mg() / 1000.0f;
  az = imu.getAccelZ_mg() / 1000.0f;


  if (millis() - lastPrint > 200)
  {

    Serial.print("Distance: ");
    Serial.print(echoSensor.getDistance());
    Serial.println(" cm");

    Serial.print("Accel (g):  ");
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.println(az, 3);

    
    Serial.print("Gyro (dps): ");
    Serial.print(gx, 3); Serial.print(", ");
    Serial.print(gy, 3); Serial.print(", ");
    Serial.println(gz, 3);
    
    Serial.println("---------------------");

    lastPrint = millis();
  }
}
