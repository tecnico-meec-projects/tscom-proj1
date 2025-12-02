#include "Pedometer.hpp"

Pedometer::Pedometer()
{
  reset();
}

void Pedometer::reset()
{

  flag_max = 0;
  flag_max_min_samplecounter = 0;
  count_steps = 0;

  IndexBuffer = 0;
  IndexAverage = 0;
  IndexWindowMin = 0;
  IndexWindowMax = 0;
  IndexThreshold = 0;

  StepToStepSamples = 0;
  Regulation_mode = 0;
  possible_steps = 0;

  NewThreshold = 0;
  old_threshold = PEDO_INIT_OFFSET_VALUE;

  BufferDinamicThreshold = PEDO_INIT_OFFSET_VALUE * PEDO_THRESHOLD_ORDER;
  flag_threshold = 0;
  flag_threshold_counter = 0;

  LastMax = 0;
  LastMin = 0;
  Difference = 0;

  ModuleData = 0;
  WindowMax = 0;
  WindowMin = PEDO_INIT_VALUE_MIN;

  FilterMeanBuffer = 0;
  FilterModuleData = 0;

  // Clear buffers
  for (int i = 0; i < PEDO_THRESHOLD_ORDER; i++)
    buffer_dynamic_threshold[i] = PEDO_INIT_OFFSET_VALUE;

  for (int i = 0; i < PEDO_FILTER_ORDER; i++)
    buffer_RawData[i] = 0;

  for (int i = 0; i < PEDO_WINDOW_SIZE; i++)
    buffer_filtered_window[i] = 0;

  // Reset distance variables
  lastStrideAmax = 0;
  lastStrideAmin = 0;
  strideLength = 0.0f;
  totalDistance = 0.0f;
}



uint32_t Pedometer::computeModuleData(int16_t ax_mg, int16_t ay_mg, int16_t az_mg)
{

  int16_t X = ax_mg * 4;
  int16_t Y = ay_mg * 4;
  int16_t Z = az_mg * 4;

  uint16_t Abs_X = (X < 0) ? -X : X;
  uint16_t Abs_Y = (Y < 0) ? -Y : Y;
  uint16_t Abs_Z = (Z < 0) ? -Z : Z;

  return Abs_X + Abs_Y + Abs_Z;
}

void Pedometer::applyFilter()
{
  FilterMeanBuffer = FilterMeanBuffer - buffer_RawData[IndexAverage] + ModuleData;
  FilterModuleData = FilterMeanBuffer / PEDO_FILTER_ORDER;
  buffer_RawData[IndexAverage] = ModuleData;
  buffer_filtered_window[IndexBuffer] = FilterModuleData;
}

void Pedometer::findWindowMaximum()
{
  WindowMax = buffer_filtered_window[0];
  IndexWindowMax = 0;
  for (int8_t i = 1; i < PEDO_WINDOW_SIZE; i++)
  {
    if (buffer_filtered_window[i] > WindowMax)
    {
      WindowMax = buffer_filtered_window[i];
      IndexWindowMax = i;
    }
  }
}

void Pedometer::findWindowMinimum()
{
  WindowMin = buffer_filtered_window[0];
  IndexWindowMin = 0;
  for (int8_t i = 1; i < PEDO_WINDOW_SIZE; i++)
  {
    if (buffer_filtered_window[i] < WindowMin)
    {
      WindowMin = buffer_filtered_window[i];
      IndexWindowMin = i;
    }
  }
}

void Pedometer::updateDynamicThreshold()
{
  NewThreshold = (LastMax + LastMin) >> 1;

  BufferDinamicThreshold =
      BufferDinamicThreshold - buffer_dynamic_threshold[IndexThreshold] + NewThreshold;
  old_threshold = BufferDinamicThreshold / PEDO_THRESHOLD_ORDER;
  buffer_dynamic_threshold[IndexThreshold] = NewThreshold;

  IndexThreshold++;
  if (IndexThreshold > PEDO_THRESHOLD_ORDER - 1)
    IndexThreshold = 0;
}

float Pedometer::calculateStrideLength(int32_t raw)
{
  if (raw <= 0)
    return 0.0f;

  float s = sqrtf((float)raw);    // first sqrt
  s *= 16.0f;                     // scale
  s = sqrtf(s);                   // second sqrt (quartic root)
  s /= 3.0f;                      // normalize
  s = (s * 7.0f) / 48.0f / 2.093f; // fudge factor
  return s;
}

void Pedometer::registerStep()
{
  // Calculate stride length
  lastStrideAmax = LastMax;
  lastStrideAmin = LastMin;
  int32_t raw = lastStrideAmax - lastStrideAmin;
  strideLength = calculateStrideLength(raw);

  // Update step count and distance
  if (Regulation_mode)
  {
    count_steps++;
    totalDistance += strideLength;
  }
  else
  {
    possible_steps++;
    if (possible_steps == 8)
    {
      count_steps += possible_steps;
      totalDistance += strideLength * possible_steps;
      possible_steps = 0;
      Regulation_mode = 1;
    }
  }
}

void Pedometer::handleMaxDetection()
{
  int8_t centerIndex = (IndexBuffer + (PEDO_WINDOW_SIZE >> 1)) % PEDO_WINDOW_SIZE;
  
  if (IndexWindowMax == centerIndex)
  {
    flag_max = 1;
    LastMax = WindowMax;
    flag_max_min_samplecounter = 0;
  }
}

void Pedometer::handleMinDetection()
{
  int8_t centerIndex = (IndexBuffer + (PEDO_WINDOW_SIZE >> 1)) % PEDO_WINDOW_SIZE;
  
  if (IndexWindowMin == centerIndex)
  {
    LastMin = WindowMin;
    Difference = LastMax - LastMin;

    flag_max = 0;
    flag_max_min_samplecounter = 0;

    // Check if signal crosses threshold
    if ((LastMax > (old_threshold + (PEDO_SENSITIVITY >> 1))) &&
        (LastMin < (old_threshold - (PEDO_SENSITIVITY >> 1))))
    {
      flag_threshold = 1;
      flag_threshold_counter = 0;
    }
    else
    {
      flag_threshold_counter++;
    }

    // Process potential step
    if (Difference > PEDO_SENSITIVITY)
    {
      updateDynamicThreshold();

      if (flag_threshold)
      {
        flag_threshold = 0;
        StepToStepSamples = 0;
        flag_max_min_samplecounter = 0;
        registerStep();
      }
    }

    WindowMin = PEDO_INIT_VALUE_MIN;

    // Threshold failed twice → reset
    if (flag_threshold_counter > 1)
    {
      flag_threshold_counter = 0;
      flag_max_min_samplecounter = 0;
      WindowMax = 0;
      WindowMin = 0;
      Regulation_mode = 0;
      possible_steps = 0;
    }
  }
  else
  {
    handleMaxMinTimeout();
  }
}

void Pedometer::handleMaxMinTimeout()
{
  flag_max_min_samplecounter++;
  if (flag_max_min_samplecounter == PEDO_ONE_SECOND)
  {
    flag_max_min_samplecounter = 0;
    WindowMax = 0;
    WindowMin = 0;
    flag_max = 0;
    possible_steps = 0;
  }
}

void Pedometer::updateCircularIndices()
{
  IndexBuffer++;
  if (IndexBuffer > PEDO_WINDOW_SIZE - 1)
    IndexBuffer = 0;

  IndexAverage++;
  if (IndexAverage > PEDO_FILTER_ORDER - 1)
    IndexAverage = 0;
}

void Pedometer::handleRegulationTimeout()
{
  StepToStepSamples++;
  if (StepToStepSamples >= PEDO_REG_OFF_TIME)
  {
    StepToStepSamples = 0;
    possible_steps = 0;
    flag_max_min_samplecounter = 0;

    if (Regulation_mode == 1)
    {
      Regulation_mode = 0;
      old_threshold = PEDO_INIT_OFFSET_VALUE;
      BufferDinamicThreshold = PEDO_INIT_OFFSET_VALUE * PEDO_THRESHOLD_ORDER;
      IndexThreshold = 0;
      flag_threshold_counter = 0;

      for (int i = 0; i < PEDO_THRESHOLD_ORDER; i++)
        buffer_dynamic_threshold[i] = PEDO_INIT_OFFSET_VALUE;
    }
  }
}



void Pedometer::update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg)
{
  //Compute acceleration magnitude
  ModuleData = computeModuleData(ax_mg, ay_mg, az_mg);

  //Apply moving average filter
  applyFilter();

  //Find max and min in sliding window
  findWindowMaximum();
  findWindowMinimum();

  //Detect peaks and valleys
  if (!flag_max)
  {
    handleMaxDetection();
  }
  else
  {
    handleMinDetection();
  }

  //Update circular buffer indices
  updateCircularIndices();

  //Handle regulation mode timeout
  handleRegulationTimeout();
}

uint32_t Pedometer::getStepCount() const
{
  return count_steps;
}

float Pedometer::getLastStrideLength() const
{
  return strideLength;
}

float Pedometer::getTotalDistance() const
{
  return totalDistance;
}

float Pedometer::getAverageStepLength() const
{
  if (count_steps == 0)
    return 0.0f;
  return totalDistance / (float)count_steps;
}