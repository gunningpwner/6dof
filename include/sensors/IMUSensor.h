#pragma once
#include "interfaces/ISensor.h"

struct IMUData {
    Vec3 accel; // m/s^2
    Vec3 gyro;  // rad/s
};

class IMUSensor : public ISensor<IMUData> {
private:
    // Noise standard deviations
    float accel_noise_std_;
    float gyro_noise_std_;

public:
    IMUSensor(float accel_noise = 0.05f, float gyro_noise = 0.005f);
    
    IMUData read(const State& truth) override;
};