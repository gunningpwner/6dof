#pragma once

#include "vehicle/Quadcopter.h"
#include "core/Scheduler.h"
#include "DataManager.h"

class MagEmulator {
public:
    MagEmulator(Quadcopter& quad, Scheduler& sched, DataManager::SensorBuffer& buffer);

private:
    void sample(Time_us current_time);

    Quadcopter& quadcopter;
    Scheduler& scheduler;
    DataManager::SensorBuffer& m_sensor_buffer;

    // Noise parameters
    float noise_std_ = 0.01f; // Gauss
    Vec3 bias_ = {0.05f, -0.05f, 0.02f}; // Constant bias, in sensor frame
};