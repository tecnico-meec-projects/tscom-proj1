#include "UltrasonicSensor.hpp"

UltrasonicSensor* UltrasonicSensor::instance = nullptr;

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
  : trigPin(trigPin), echoPin(echoPin) {}

void UltrasonicSensor::begin()
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  lastTriggerTime = micros();
  instance = this;
  attachInterrupt(digitalPinToInterrupt(echoPin), UltrasonicSensor::isrHandler0, CHANGE);
}

void UltrasonicSensor::update()
{
  unsigned long now = micros();

  if (now - lastTriggerTime >= triggerInterval) {
    lastTriggerTime = now;
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  }

  if (pulseDone) {
    unsigned long duration = pulseEnd - pulseStart;
    pulseDone = false;
    distance = (duration * 0.0343f) / 2.0f; // cm
  }
}

float UltrasonicSensor::getDistance() const
{
  return distance;
}

void UltrasonicSensor::isrHandler0()
{
  if (instance) instance->handleISR();
}

void UltrasonicSensor::handleISR()
{
  if (digitalRead(echoPin) == HIGH)
    pulseStart = micros();
  else {
    pulseEnd = micros();
    pulseDone = true;
  }
}
