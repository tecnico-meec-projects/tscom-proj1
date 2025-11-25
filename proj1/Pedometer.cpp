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
  FilterMeanBuffer = 0;
  FilterModuleData = 0;
  StepToStepSamples = 0;
  Regulation_mode = 0;
  possible_steps = 0;
  NewThreshold = 0;
  LastMax = 0;
  LastMin = 0;
  Difference = 0;
  ModuleData = 0;
  old_threshold = PEDO_INIT_OFFSET_VALUE;
  BufferDinamicThreshold = PEDO_INIT_OFFSET_VALUE * PEDO_THRESHOLD_ORDER;
  flag_threshold = 0;
  flag_threshold_counter = 0;

  for (int i = 0; i < PEDO_THRESHOLD_ORDER; i++) {
    buffer_dynamic_threshold[i] = PEDO_INIT_OFFSET_VALUE;
  }
  for (int i = 0; i < PEDO_FILTER_ORDER; i++) {
    buffer_RawData[i] = 0;
  }
  for (int i = 0; i < PEDO_WINDOW_SIZE; i++) {
    buffer_filtered_window[i] = 0;
  }
}

void Pedometer::update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg)
{

  int16_t X = (int16_t)((int32_t)ax_mg * 4);
  int16_t Y = (int16_t)((int32_t)ay_mg * 4);
  int16_t Z = (int16_t)((int32_t)az_mg * 4);

  uint16_t Abs_X = (X < 0) ? -X : X;
  uint16_t Abs_Y = (Y < 0) ? -Y : Y;
  uint16_t Abs_Z = (Z < 0) ? -Z : Z;

  ModuleData = (uint32_t)Abs_X + Abs_Y + Abs_Z;


  FilterMeanBuffer = FilterMeanBuffer - buffer_RawData[IndexAverage] + ModuleData;
  FilterModuleData = FilterMeanBuffer / PEDO_FILTER_ORDER;
  buffer_RawData[IndexAverage] = ModuleData;

  buffer_filtered_window[IndexBuffer] = FilterModuleData;


  WindowMax = buffer_filtered_window[0];
  IndexWindowMax = 0;
  for (int8_t i = 1; i < PEDO_WINDOW_SIZE; i++) {
    if (buffer_filtered_window[i] > WindowMax) {
      WindowMax = buffer_filtered_window[i];
      IndexWindowMax = i;
    }
  }


  WindowMin = buffer_filtered_window[0];
  IndexWindowMin = 0;
  for (int8_t i = 1; i < PEDO_WINDOW_SIZE; i++) {
    if (buffer_filtered_window[i] < WindowMin) {
      WindowMin = buffer_filtered_window[i];
      IndexWindowMin = i;
    }
  }


  if (flag_max == 0) {

    if (IndexWindowMax == ((IndexBuffer + (PEDO_WINDOW_SIZE >> 1)) % PEDO_WINDOW_SIZE)) {
      flag_max = 1;
      LastMax = WindowMax;
      flag_max_min_samplecounter = 0;
    }
  } else {

    if (IndexWindowMin == ((IndexBuffer + (PEDO_WINDOW_SIZE >> 1)) % PEDO_WINDOW_SIZE)) {
      LastMin = WindowMin;
      Difference = LastMax - LastMin;
      flag_max = 0;
      flag_max_min_samplecounter = 0;


      if ((LastMax > (old_threshold + (PEDO_SENSITIVITY >> 1))) &&
          (LastMin < (old_threshold - (PEDO_SENSITIVITY >> 1)))) {
        flag_threshold = 1;
        flag_threshold_counter = 0;
      } else {
        flag_threshold_counter++;
      }


      if (Difference > PEDO_SENSITIVITY) {
        NewThreshold = (LastMax + LastMin) >> 1;
        BufferDinamicThreshold =
            BufferDinamicThreshold - buffer_dynamic_threshold[IndexThreshold] + NewThreshold;
        old_threshold = BufferDinamicThreshold / PEDO_THRESHOLD_ORDER;
        buffer_dynamic_threshold[IndexThreshold] = NewThreshold;

        IndexThreshold++;
        if (IndexThreshold > PEDO_THRESHOLD_ORDER - 1) {
          IndexThreshold = 0;
        }


        if (flag_threshold) {
          flag_threshold = 0;
          StepToStepSamples = 0;
          flag_max_min_samplecounter = 0;

          if (Regulation_mode) {

            count_steps++;
          } else {

            possible_steps++;
            if (possible_steps == 8) {
              count_steps += possible_steps;
              possible_steps = 0;
              Regulation_mode = 1;
            }
          }
        }
      }

      WindowMin = PEDO_INIT_VALUE_MIN;


      if (flag_threshold_counter > 1) {
        flag_threshold_counter = 0;
        flag_max_min_samplecounter = 0;
        WindowMax = 0;
        WindowMin = 0;
        Regulation_mode = 0;
        possible_steps = 0;
        flag_threshold_counter = 0;
      }
    } else {

      flag_max_min_samplecounter++;
      if (flag_max_min_samplecounter == PEDO_ONE_SECOND) {
        flag_max_min_samplecounter = 0;
        WindowMax = 0;
        WindowMin = 0;
        flag_max = 0;
        possible_steps = 0;
      }
    }
  }


  IndexBuffer++;
  if (IndexBuffer > PEDO_WINDOW_SIZE - 1) {
    IndexBuffer = 0;
  }

  IndexAverage++;
  if (IndexAverage > PEDO_FILTER_ORDER - 1) {
    IndexAverage = 0;
  }


  StepToStepSamples++;
  if (StepToStepSamples >= PEDO_REG_OFF_TIME) {
    StepToStepSamples = 0;
    possible_steps = 0;
    flag_max_min_samplecounter = 0;

    if (Regulation_mode == 1) {
      Regulation_mode = 0;
      old_threshold = PEDO_INIT_OFFSET_VALUE;
      BufferDinamicThreshold = PEDO_INIT_OFFSET_VALUE * PEDO_THRESHOLD_ORDER;
      IndexThreshold = 0;
      flag_threshold_counter = 0;
      for (int i = 0; i < PEDO_THRESHOLD_ORDER; i++) {
        buffer_dynamic_threshold[i] = PEDO_INIT_OFFSET_VALUE;
      }
    }
  }
}

uint32_t Pedometer::getStepCount() const
{
  return count_steps;
}