#include "sensors/GPSSensor.h"
#include <random> // Heavy include! Keep it in the CPP.

// Static random engine (so we don't re-seed every time we read)
static std::default_random_engine generator;
static std::normal_distribution<double> distribution(0.0, 1.0);

GPSSensor::GPSSensor(std::shared_ptr<IWorldModel> w, double h, double v) 
    : world_(w), noise_sigma_h_(h), noise_sigma_v_(v) {}

GPSData GPSSensor::read(const State& truth) {
    // 1. Get Ideal Measurement
    GeodeticPos perfect_pos = world_->nedToLLA(truth.pos_ned);

    // 2. Add Noise (The "Sensor Logic")
    // Convert meters noise to radians (approx)
    double lat_noise = (distribution(generator) * noise_sigma_h_) / 6371000.0;
    double lon_noise = (distribution(generator) * noise_sigma_h_) / (6371000.0 * cos(perfect_pos.lat_rad));
    double alt_noise = distribution(generator) * noise_sigma_v_;

    return {
        perfect_pos.lat_rad + lat_noise,
        perfect_pos.lon_rad + lon_noise,
        perfect_pos.alt_m + alt_noise
    };
}