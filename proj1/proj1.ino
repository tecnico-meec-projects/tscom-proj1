#include "UltrasonicSensor.hpp"

//PINS
static constexpr uint8_t trigPin = 2;
static constexpr uint8_t echoPin = 3;

UltrasonicSensor echo(trigPin, echoPin);

unsigned long startTime;

void setup()
{
  Serial.begin(9600);
  echo.begin();
  startTime = millis();
}

void loop()
{
  echo.update();

  if (millis() - startTime > 100)
  {
    Serial.print("Distance: ");
    Serial.print(echo.getDistance());
    Serial.println(" cm");
    startTime = millis();
  }
}