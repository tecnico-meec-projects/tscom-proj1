#ifndef INERTIAL_SPEED_ESTIMATOR_HPP
#define INERTIAL_SPEED_ESTIMATOR_HPP

#include <Arduino.h>

class InertialSpeedEstimator
{
public:

    explicit InertialSpeedEstimator(float samplePeriod_s = 0.02f);

    void reset();


    void startCalibration();
    void addCalibrationSample(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);
    void finishCalibration();


    void update(int16_t ax_mg, int16_t ay_mg, int16_t az_mg);


    float getSpeedMps() const;
    float getSpeedKmh() const;


    float getEstimatedG() const { return g_est; }

private:
    static constexpr float MG_TO_MPS2  = 9.80665f / 1000.0f; // 1 mg → m/s²
    static constexpr float DEFAULT_G   = 9.80665f;


    static constexpr float ALPHA_MAG   = 0.2f;  
    static constexpr float NOISE_FLOOR = 0.10f; 
    static constexpr float DAMPING     = 0.99f; 

    float Ts;            
    float speed_mps;   
    float a_mag_filt;   


    bool  calibrating;
    bool  calibrated;
    float g_est;         
    uint32_t calibSamples;
    float calibSumMag;
};

#endif // INERTIAL_SPEED_ESTIMATOR_HPP
