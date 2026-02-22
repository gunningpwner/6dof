#include "sensors/IMUSensor.h"
#include <random>

// Static random engine
static std::default_random_engine gen;
static std::normal_distribution<float> dist(0.0f, 1.0f);

IMUSensor::IMUSensor(float accel_noise, float gyro_noise) 
    : accel_noise_std_(accel_noise), gyro_noise_std_(gyro_noise) {}

IMUData IMUSensor::read(const State& truth) {
    IMUData data;
    
    // 1. Base Truth
    data.accel = truth.accel_body;
    data.gyro = truth.omega_body;

    // 2. Add White Noise
    Vec3 accel_noise(dist(gen) * accel_noise_std_, 
                     dist(gen) * accel_noise_std_, 
                     dist(gen) * accel_noise_std_);

    Vec3 gyro_noise(dist(gen) * gyro_noise_std_, 
                    dist(gen) * gyro_noise_std_, 
                    dist(gen) * gyro_noise_std_);

    data.accel += accel_noise;
    data.gyro += gyro_noise;

    return data;
}