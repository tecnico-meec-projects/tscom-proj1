#ifndef VELOCITY_ESTIMATOR_HPP
#define VELOCITY_ESTIMATOR_HPP

#include <Arduino.h>

class VelocityEstimator
{
public:
  VelocityEstimator();

  /// Limpa velocidade, bias, etc.
  void reset();

  /// Inicia calibração (deve ser chamado com o sensor parado)
  void startCalibration();

  /// Adiciona uma amostra de calibração (sensor parado)
  void addCalibrationSample(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);

  /// Termina calibração e calcula o vetor de bias/gravidade
  void finishCalibration();

  /// Atualiza o estimador com nova aceleração em mg (Nano 33 BLE IMU)
  /// Chamar regularmente (ex: a cada iteração do loop)
  void update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);

  /// Velocidade total (módulo) em m/s
  float getSpeed() const;

  /// Componentes da velocidade em m/s (no referencial do sensor)
  float getVx() const { return vx; }
  float getVy() const { return vy; }
  float getVz() const { return vz; }

private:
  static constexpr float MG_TO_MPS2 = 9.80665f / 1000.0f; // 1 mg → m/s²

  // Estado de calibração
  bool     isCalibrating;
  bool     isCalibrated;
  uint32_t calibSamples;
  float    calibSumAx_mps2;
  float    calibSumAy_mps2;
  float    calibSumAz_mps2;

  // Bias (inclui gravidade) em m/s²
  float ax_bias_mps2;
  float ay_bias_mps2;
  float az_bias_mps2;

  // Velocidade integrada em m/s
  float vx, vy, vz;

  // Tempo da última atualização (micros)
  uint32_t lastMicros;

  // Pequenos truques para limitar drift
  static constexpr float ACCEL_DEADBAND = 0.05f;  // m/s²
  static constexpr float VEL_DAMPING    = 0.999f; // factor de amortecimento
};

#endif // VELOCITY_ESTIMATOR_HPP