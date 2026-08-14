#pragma once

#include "vehicle/Quadcopter.h"
#include "core/Scheduler.h"
#include "DataManager.h"

class GPSEmulator {
public:
    GPSEmulator(Quadcopter& quad, Scheduler& sched, DataManager::SensorBuffer& buffer);

private:
    void sample(Time_us current_time);

    Quadcopter& quadcopter;
    Scheduler& scheduler;
    DataManager::SensorBuffer& m_sensor_buffer;

    // Noise parameters
    double noise_sigma_h_ = 0; // Horizontal noise (m)
    double noise_sigma_v_ = 0; // Vertical noise (m)
};