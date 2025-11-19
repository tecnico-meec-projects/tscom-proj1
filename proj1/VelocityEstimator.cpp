#include "VelocityEstimator.hpp"

VelocityEstimator::VelocityEstimator()
{
  reset();
}

void VelocityEstimator::reset()
{
  isCalibrating  = false;
  isCalibrated   = false;
  calibSamples   = 0;
  calibSumAx_mps2 = 0.0f;
  calibSumAy_mps2 = 0.0f;
  calibSumAz_mps2 = 0.0f;

  ax_bias_mps2 = 0.0f;
  ay_bias_mps2 = 0.0f;
  az_bias_mps2 = 0.0f;

  vx = vy = vz = 0.0f;
  lastMicros = 0;
}

void VelocityEstimator::startCalibration()
{
  isCalibrating   = true;
  isCalibrated    = false;
  calibSamples    = 0;
  calibSumAx_mps2 = 0.0f;
  calibSumAy_mps2 = 0.0f;
  calibSumAz_mps2 = 0.0f;
}

void VelocityEstimator::addCalibrationSample(int16_t ax_mg, int16_t ay_mg, int16_t az_mg)
{
  if (!isCalibrating)
    return;

  float ax = ax_mg * MG_TO_MPS2;
  float ay = ay_mg * MG_TO_MPS2;
  float az = az_mg * MG_TO_MPS2;

  calibSumAx_mps2 += ax;
  calibSumAy_mps2 += ay;
  calibSumAz_mps2 += az;
  calibSamples++;
}

void VelocityEstimator::finishCalibration()
{
  if (!isCalibrating || calibSamples == 0)
    return;

  ax_bias_mps2 = calibSumAx_mps2 / (float)calibSamples;
  ay_bias_mps2 = calibSumAy_mps2 / (float)calibSamples;
  az_bias_mps2 = calibSumAz_mps2 / (float)calibSamples;

  isCalibrating = false;
  isCalibrated  = true;
}

void VelocityEstimator::update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg)
{
  uint32_t now = micros();

  if (lastMicros == 0) {
    lastMicros = now;
    return; // ainda não temos dt
  }

  float dt = (now - lastMicros) / 1e6f; // segundos
  lastMicros = now;

  if (dt <= 0.0f || dt > 0.5f) {
    // salto de tempo estranho → não integrar
    return;
  }

  // Converte para m/s²
  float ax = ax_mg * MG_TO_MPS2;
  float ay = ay_mg * MG_TO_MPS2;
  float az = az_mg * MG_TO_MPS2;

  // Se não estiver calibrado, assume bias = 0 (velocidade vai derivar mais)
  float ax_corr = ax - (isCalibrated ? ax_bias_mps2 : 0.0f);
  float ay_corr = ay - (isCalibrated ? ay_bias_mps2 : 0.0f);
  float az_corr = az - (isCalibrated ? az_bias_mps2 : 0.0f);

  // Deadband – pequenos ruídos à volta de 0 ficam a 0
  if (fabs(ax_corr) < ACCEL_DEADBAND) ax_corr = 0.0f;
  if (fabs(ay_corr) < ACCEL_DEADBAND) ay_corr = 0.0f;
  if (fabs(az_corr) < ACCEL_DEADBAND) az_corr = 0.0f;

  // Integração simples: v = v + a * dt
  vx += ax_corr * dt;
  vy += ay_corr * dt;
  vz += az_corr * dt;

  // Amortecimento leve para limitar drift infinito
  vx *= VEL_DAMPING;
  vy *= VEL_DAMPING;
  vz *= VEL_DAMPING;
}

float VelocityEstimator::getSpeed() const
{
  return sqrtf(vx * vx + vy * vy + vz * vz);
}