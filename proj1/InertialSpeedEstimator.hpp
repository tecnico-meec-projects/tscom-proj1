#ifndef INERTIAL_SPEED_ESTIMATOR_HPP
#define INERTIAL_SPEED_ESTIMATOR_HPP

#include <Arduino.h>

class InertialSpeedEstimator
{
public:
    // samplePeriod_s: período de amostragem (por ex. 0.02 para 50 Hz)
    explicit InertialSpeedEstimator(float samplePeriod_s = 0.02f);

    void reset();

    // --- Calibração da gravidade (sensor parado) ---
    void startCalibration();
    void addCalibrationSample(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);
    void finishCalibration();

    // Atualiza o estimador com uma nova amostra (em mg)
    // Assume que é chamado aproximadamente a cada samplePeriod_s
    void update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);

    // Velocidade estimada [m/s] e [km/h]
    float getSpeedMps() const;
    float getSpeedKmh() const;

    // Ler g estimado (só para debug)
    float getEstimatedG() const { return g_est; }

private:
    static constexpr float MG_TO_MPS2  = 9.80665f / 1000.0f; // 1 mg → m/s²
    static constexpr float DEFAULT_G   = 9.80665f;

    // Parâmetros de filtragem
    static constexpr float ALPHA_MAG   = 0.2f;  // filtro low-pass para |a|
    static constexpr float NOISE_FLOOR = 0.10f; // m/s² abaixo disto é ruído
    static constexpr float DAMPING     = 0.99f; // travão por amostra

    float Ts;            // período de amostragem (s)
    float speed_mps;     // velocidade atual (m/s)
    float a_mag_filt;    // módulo da aceleração filtrado (m/s²)

    // Calibração da gravidade
    bool  calibrating;
    bool  calibrated;
    float g_est;         // gravidade estimada (m/s²) a partir do módulo médio
    uint32_t calibSamples;
    float calibSumMag;
};

#endif // INERTIAL_SPEED_ESTIMATOR_HPP
