#include "StepSizeEstimator.hpp"

StepSizeEstimator::StepSizeEstimator()
: lastStepCount(0),
  lastUpdateTime(0),
  currentStepSize(0.0f),
  smoothedStepSize(0.0f),
  bufferIndex(0),
  bufferFilled(0),
  stepFrequency(0.0f),
  initialized(false)
{
    for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
        stepSizeBuffer[i] = 0.0f;
    }
}

void StepSizeEstimator::reset()
{
    lastStepCount = 0;
    lastUpdateTime = 0;
    currentStepSize = 0.0f;
    smoothedStepSize = 0.0f;
    bufferIndex = 0;
    bufferFilled = 0;
    stepFrequency = 0.0f;
    initialized = false;
    
    for (uint8_t i = 0; i < WINDOW_SIZE; i++) {
        stepSizeBuffer[i] = 0.0f;
    }
}

void StepSizeEstimator::update(float speedMps, uint32_t stepCount)
{
    unsigned long currentTime = millis();
    

    if (!initialized) {
        lastStepCount = stepCount;
        lastUpdateTime = currentTime;
        initialized = true;
        return;
    }
    

    unsigned long deltaTime = currentTime - lastUpdateTime;
    if (deltaTime < 100) return; // update at most every 100ms
    

    uint32_t newSteps = stepCount - lastStepCount;
    

    if (newSteps > 0 && speedMps >= MIN_SPEED) {
        // Calculate step frequency (cadence)
        float deltaTimeSec = deltaTime / 1000.0f;
        stepFrequency = newSteps / deltaTimeSec;
        
        // Calculate step size: distance = speed * time, step_size = distance / steps
        float distance = speedMps * deltaTimeSec;
        currentStepSize = distance / newSteps;
        
        // Validate and constrain step size to reasonable values
        if (currentStepSize < MIN_STEP_SIZE) {
            currentStepSize = MIN_STEP_SIZE;
        } else if (currentStepSize > MAX_STEP_SIZE) {
            currentStepSize = MAX_STEP_SIZE;
        }
        
        // Add to circular buffer
        stepSizeBuffer[bufferIndex] = currentStepSize;
        bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
        if (bufferFilled < WINDOW_SIZE) {
            bufferFilled++;
        }
        
        // Apply exponential smoothing
        if (smoothedStepSize == 0.0f) {
            smoothedStepSize = currentStepSize;
        } else {
            smoothedStepSize = ALPHA * currentStepSize + (1.0f - ALPHA) * smoothedStepSize;
        }
        
        // Update for next iteration
        lastStepCount = stepCount;
        lastUpdateTime = currentTime;
    } else if (speedMps < MIN_SPEED) {
        // If speed is too low, gradually decay the step frequency
        stepFrequency *= 0.95f;
        if (stepFrequency < 0.05f) {
            stepFrequency = 0.0f;
        }
    }
}

float StepSizeEstimator::getStepSizeMeters() const
{
    return smoothedStepSize;
}

float StepSizeEstimator::getStepSizeCm() const
{
    return smoothedStepSize * 100.0f;
}

float StepSizeEstimator::getAverageStepSizeMeters() const
{
    if (bufferFilled == 0) return 0.0f;
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < bufferFilled; i++) {
        sum += stepSizeBuffer[i];
    }
    
    return sum / bufferFilled;
}

float StepSizeEstimator::getCadence() const
{
    return stepFrequency;
}