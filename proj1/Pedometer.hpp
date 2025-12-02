#ifndef PEDOMETER_HPP
#define PEDOMETER_HPP

#include <Arduino.h>


#define PEDO_THRESHOLD_ORDER   4
#define PEDO_FILTER_ORDER      4
#define PEDO_WINDOW_SIZE       ((PEDO_FILTER_ORDER * 4) + 1)

#define PEDO_SENSITIVITY       400   
#define PEDO_INIT_OFFSET_VALUE 4000  
#define PEDO_INIT_VALUE_MIN    0

// ODR alvo = 50 Hz
#define PEDO_ONE_SECOND        50
#define PEDO_REG_OFF_TIME      (PEDO_ONE_SECOND * 2)   
#define PEDO_DISTANCE_K        0.00025f  

class Pedometer
{
public:
  Pedometer();


  void reset();


  void update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);


  uint32_t getStepCount() const;


  float getLastStrideLength() const;


  float getTotalDistance() const;


  float getAverageStepLength() const;

private:

  uint32_t computeModuleData(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);
  void applyFilter();
  void findWindowMaximum();
  void findWindowMinimum();
  void updateDynamicThreshold();
  float calculateStrideLength(int32_t raw);
  void registerStep();
  void handleMaxDetection();
  void handleMinDetection();
  void handleMaxMinTimeout();
  void updateCircularIndices();
  void handleRegulationTimeout();


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


  int32_t lastStrideAmax;
  int32_t lastStrideAmin;

  float strideLength;   
  float totalDistance;  
};

#endif // PEDOMETER_HPP