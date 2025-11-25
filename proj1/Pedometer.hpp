#ifndef PEDOMETER_HPP
#define PEDOMETER_HPP

#include <Arduino.h>


#define PEDO_THRESHOLD_ORDER   4
#define PEDO_FILTER_ORDER      4
#define PEDO_WINDOW_SIZE       ((PEDO_FILTER_ORDER * 4) + 1)

#define PEDO_SENSITIVITY       400   
#define PEDO_INIT_OFFSET_VALUE 4000  
#define PEDO_INIT_VALUE_MIN    0

// ODR  = 50 Hz
#define PEDO_ONE_SECOND        50
#define PEDO_REG_OFF_TIME      (PEDO_ONE_SECOND * 2)   

class Pedometer
{
public:
  Pedometer();


  void reset();


  void update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);


  uint32_t getStepCount() const;

private:

  int32_t buffer_RawData[PEDO_FILTER_ORDER];
  int32_t buffer_filtered_window[PEDO_WINDOW_SIZE];
  int32_t buffer_dynamic_threshold[PEDO_THRESHOLD_ORDER];

  uint32_t count_steps;

  int8_t IndexWindowMin, IndexWindowMax;
  int8_t IndexThreshold, IndexBuffer, IndexAverage;
  int8_t StepToStepSamples, Regulation_mode;
  int8_t flag_max_min_samplecounter, possible_steps;

  uint8_t flag_max, flag_threshold, flag_threshold_counter;

  uint32_t LastMax, LastMin, WindowMin, WindowMax;
  uint32_t FilterMeanBuffer, FilterModuleData, ModuleData;
  uint32_t Difference, BufferDinamicThreshold, NewThreshold, old_threshold;
};

#endif // PEDOMETER_HPP