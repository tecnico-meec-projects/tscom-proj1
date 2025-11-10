#pragma once
#include <Arduino.h>

class UltrasonicSensor
{
  public:
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin);

    void  begin();
    void  update();
    float getDistance() const;

  private:
    static void isrHandler0();
    void handleISR();

    uint8_t trigPin;
    uint8_t echoPin;

    volatile unsigned long pulseStart = 0;
    volatile unsigned long pulseEnd = 0;
    volatile bool pulseDone = false;

    float distance = 0.0;
    unsigned long lastTriggerTime = 0;
    static constexpr unsigned long triggerInterval = 10000; //us

    static UltrasonicSensor* instance;
};
