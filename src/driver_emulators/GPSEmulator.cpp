#include "driver_emulators/GPSEmulator.h"
#include "interfaces/IWorldModel.h"
#include "dynamics/QuadDynamics.h"
#include <random>
#include <math.h>
// Static random engine for GPS
static std::default_random_engine gps_emu_gen;
static std::normal_distribution<double> gps_emu_dist(0.0, 1.0);

GPSEmulator::GPSEmulator(Quadcopter& quad, Scheduler& sched, DataManager::SensorBuffer& buffer)
    : quadcopter(quad), scheduler(sched), m_sensor_buffer(buffer) {
    // Schedule the first sample. GPS updates are slow, e.g., 5Hz -> 200ms
    scheduler.schedule(200000, [this](Time_us t){ this->sample(t); });
}

void GPSEmulator::sample(Time_us current_time) {
    // 1. Propagate physics to EXACTLY right now
    quadcopter.propagate_to(current_time);

    // 2. Get truth state
    SimState truth = quadcopter.getTruth();
    
    // 3. Get world model to convert NED to LLA
    auto world_model = quadcopter.getDynamics()->getWorld();

    // 4. Get Ideal Measurement
    GeodeticPos perfect_pos = world_model->nedToLLA(truth.pos_ned);

    // 5. Add Noise
    double lat_noise = (gps_emu_dist(gps_emu_gen) * noise_sigma_h_) / 6371000.0;
    double lon_noise = (gps_emu_dist(gps_emu_gen) * noise_sigma_h_) / (6371000.0 * cos(perfect_pos.lat_rad));
    double alt_noise = gps_emu_dist(gps_emu_gen) * noise_sigma_v_;

    // 6. Get a buffer from DataManager
    SensorData* gps_data = m_sensor_buffer.claim();
    if (gps_data) {
        gps_data->sensor = SensorData::Type::GPS;
        gps_data->timestamp = current_time;
        gps_data->data.gps.lla[0] = perfect_pos.lat_rad*180.0/M_PI + lat_noise;
        gps_data->data.gps.lla[1] = perfect_pos.lon_rad*180.0/M_PI + lon_noise;
        gps_data->data.gps.lla[2] = perfect_pos.alt_m + alt_noise;
        
        // Simulate velocity with some noise too
        gps_data->data.gps.vel[0] = truth.vel_ned.x() + (gps_emu_dist(gps_emu_gen) * 0.1f);
        gps_data->data.gps.vel[1] = truth.vel_ned.y() + (gps_emu_dist(gps_emu_gen) * 0.1f);
        gps_data->data.gps.vel[2] = truth.vel_ned.z() + (gps_emu_dist(gps_emu_gen) * 0.1f);
        
        m_sensor_buffer.commit(gps_data);
    }

    // 7. Schedule the next sample (e.g., 5Hz -> 200ms)
    scheduler.schedule(200000, [this](Time_us t){ this->sample(t); });
}