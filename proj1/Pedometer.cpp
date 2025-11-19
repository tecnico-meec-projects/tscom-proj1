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
  // Escala mg → “LSB” tipo ADXL367 (1 g ≈ 4000)
  // IMU dá ~1000 mg → multiplicamos por 4 → ~4000
  int16_t X = (int16_t)((int32_t)ax_mg * 4);
  int16_t Y = (int16_t)((int32_t)ay_mg * 4);
  int16_t Z = (int16_t)((int32_t)az_mg * 4);

  uint16_t Abs_X = (X < 0) ? -X : X;
  uint16_t Abs_Y = (Y < 0) ? -Y : Y;
  uint16_t Abs_Z = (Z < 0) ? -Z : Z;

  ModuleData = (uint32_t)Abs_X + Abs_Y + Abs_Z;

  // ----------------- FILTRO DA ADI -----------------
  FilterMeanBuffer = FilterMeanBuffer - buffer_RawData[IndexAverage] + ModuleData;
  FilterModuleData = FilterMeanBuffer / PEDO_FILTER_ORDER;
  buffer_RawData[IndexAverage] = ModuleData;

  buffer_filtered_window[IndexBuffer] = FilterModuleData;

  // ----------------- PROCURA DE MÁXIMO -----------------
  WindowMax = buffer_filtered_window[0];
  IndexWindowMax = 0;
  for (int8_t i = 1; i < PEDO_WINDOW_SIZE; i++) {
    if (buffer_filtered_window[i] > WindowMax) {
      WindowMax = buffer_filtered_window[i];
      IndexWindowMax = i;
    }
  }

  // ----------------- PROCURA DE MÍNIMO -----------------
  WindowMin = buffer_filtered_window[0];
  IndexWindowMin = 0;
  for (int8_t i = 1; i < PEDO_WINDOW_SIZE; i++) {
    if (buffer_filtered_window[i] < WindowMin) {
      WindowMin = buffer_filtered_window[i];
      IndexWindowMin = i;
    }
  }

  // ----------------- DETEÇÃO MAX/MIN -----------------
  if (flag_max == 0) {
    // Procurar um máximo no centro da janela
    if (IndexWindowMax == ((IndexBuffer + (PEDO_WINDOW_SIZE >> 1)) % PEDO_WINDOW_SIZE)) {
      flag_max = 1;
      LastMax = WindowMax;
      flag_max_min_samplecounter = 0;
    }
  } else {
    // Já temos um máximo → procurar um mínimo
    if (IndexWindowMin == ((IndexBuffer + (PEDO_WINDOW_SIZE >> 1)) % PEDO_WINDOW_SIZE)) {
      LastMin = WindowMin;
      Difference = LastMax - LastMin;
      flag_max = 0;
      flag_max_min_samplecounter = 0;

      // Verificar se estamos dentro da "janela" de threshold
      if ((LastMax > (old_threshold + (PEDO_SENSITIVITY >> 1))) &&
          (LastMin < (old_threshold - (PEDO_SENSITIVITY >> 1)))) {
        flag_threshold = 1;
        flag_threshold_counter = 0;
      } else {
        flag_threshold_counter++;
      }

      // ----------------- ATUALIZAÇÃO DO THRESHOLD -----------------
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

        // ----------------- CERTIFICAÇÃO DO PASSO -----------------
        if (flag_threshold) {
          flag_threshold = 0;
          StepToStepSamples = 0;
          flag_max_min_samplecounter = 0;

          if (Regulation_mode) {
            // Já estamos em modo "estável" → contar passo diretamente
            count_steps++;
          } else {
            // Modo de regulação: só quando tivermos 8 passos seguidos
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

      // Se o threshold falhar 2x seguidas, provavelmente não é andar
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
      // Entre MAX e MIN demorou demasiado → descartar
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

  // Índices circulares
  IndexBuffer++;
  if (IndexBuffer > PEDO_WINDOW_SIZE - 1) {
    IndexBuffer = 0;
  }

  IndexAverage++;
  if (IndexAverage > PEDO_FILTER_ORDER - 1) {
    IndexAverage = 0;
  }

  // Time-out para sair de Regulation Mode se estiver parado
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