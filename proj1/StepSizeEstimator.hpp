#ifndef STEP_SIZE_ESTIMATOR_HPP
#define STEP_SIZE_ESTIMATOR_HPP

#include <Arduino.h>

class StepSizeEstimator
{
public:
    StepSizeEstimator();

    void reset();


    void update(float speedMps, uint32_t stepCount);


    float getStepSizeMeters() const;


    float getStepSizeCm() const;


    float getAverageStepSizeMeters() const;


    float getCadence() const;

private:
    static constexpr uint8_t WINDOW_SIZE = 10;  // samples for averaging
    static constexpr float MIN_SPEED = 0.1f;     // minimum speed to calculate (m/s) ~0.36 km/h
    static constexpr float MAX_STEP_SIZE = 1.5f; // maximum reasonable step size (m)
    static constexpr float MIN_STEP_SIZE = 0.05f; // minimum reasonable step size (m) - 5cm for shuffling
    static constexpr float ALPHA = 0.3f;         // smoothing factor

    uint32_t lastStepCount;
    uint32_t lastUpdateTime;
    
    float currentStepSize;      // current instantaneous step size (m)
    float smoothedStepSize;     // smoothed step size (m)
    
    // Circular buffer for averaging
    float stepSizeBuffer[WINDOW_SIZE];
    uint8_t bufferIndex;
    uint8_t bufferFilled;
    
    float stepFrequency;        // steps per second (cadence)
    
    bool initialized;
};

#endif // STEP_SIZE_ESTIMATOR_HPP