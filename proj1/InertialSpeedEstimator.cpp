#include "InertialSpeedEstimator.hpp"
#include <math.h>

InertialSpeedEstimator::InertialSpeedEstimator(float samplePeriod_s)
: Ts(samplePeriod_s),
speed_mps(0.0f),
a_mag_filt(0.0f),
calibrating(false),
calibrated(false),
g_est(DEFAULT_G),
calibSamples(0),
calibSumMag(0.0f)
{
}

void InertialSpeedEstimator::reset()
{
    speed_mps    = 0.0f;
    a_mag_filt   = 0.0f;
    calibrating  = false;
    calibrated   = false;
    g_est        = DEFAULT_G;
    calibSamples = 0;
    calibSumMag  = 0.0f;
}

void InertialSpeedEstimator::startCalibration()
{
    calibrating  = true;
    calibrated   = false;
    calibSamples = 0;
    calibSumMag  = 0.0f;
}

void InertialSpeedEstimator::addCalibrationSample(int16_t ax_mg, int16_t ay_mg, int16_t az_mg)
{
    if (!calibrating)
        return;

    float ax = ax_mg * MG_TO_MPS2;
    float ay = ay_mg * MG_TO_MPS2;
    float az = az_mg * MG_TO_MPS2;

    float mag = sqrtf(ax*ax + ay*ay + az*az);

    calibSumMag  += mag;
    calibSamples += 1;
}

void InertialSpeedEstimator::finishCalibration()
{
    if (!calibrating || calibSamples == 0)
        return;

    g_est = calibSumMag / (float)calibSamples;

    calibrating  = false;
    calibrated   = true;
    calibSamples = 0;
    calibSumMag  = 0.0f;
}

void InertialSpeedEstimator::update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg)
{
    // Converte para m/s²
    float ax = ax_mg * MG_TO_MPS2;
    float ay = ay_mg * MG_TO_MPS2;
    float az = az_mg * MG_TO_MPS2;

    // Módulo da aceleração total
    float mag = sqrtf(ax*ax + ay*ay + az*az);

    // Low-pass no módulo
    a_mag_filt = ALPHA_MAG * mag + (1.0f - ALPHA_MAG) * a_mag_filt;

    float g_used = calibrated ? g_est : DEFAULT_G;

    // Aproximação de aceleração linear = excesso acima da gravidade
    float a_lin = a_mag_filt - g_used;

    // Ruído pequeno → 0
    if (fabsf(a_lin) < NOISE_FLOOR)
        a_lin = 0.0f;

    // Integração simples v = v + a * Ts (apenas positiva, assumindo movimento para a frente)
    if (a_lin > 0.0f)
        speed_mps += a_lin * Ts;

    // Travão numérico para parar gradualmente quando a_lin ~ 0
    speed_mps *= DAMPING;

    // Limitações razoáveis
    if (speed_mps < 0.0f) speed_mps = 0.0f;
    if (speed_mps > 10.0f) speed_mps = 10.0f; // ~36 km/h
}

float InertialSpeedEstimator::getSpeedMps() const
{
    return speed_mps;
}

float InertialSpeedEstimator::getSpeedKmh() const
{
    return speed_mps * 3.6f;
}
