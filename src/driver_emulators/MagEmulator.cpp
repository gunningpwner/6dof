#include "driver_emulators/MagEmulator.h"
#include "interfaces/IWorldModel.h"
#include "dynamics/QuadDynamics.h"
#include <random>

// Static random engine for Mag
static std::default_random_engine mag_emu_gen;
static std::normal_distribution<float> mag_emu_dist(0.0f, 1.0f);

MagEmulator::MagEmulator(Quadcopter& quad, Scheduler& sched, DataManager::SensorBuffer& buffer)
    : quadcopter(quad), scheduler(sched), m_sensor_buffer(buffer) {
    // Schedule the first sample. Mag updates are faster than GPS, e.g., 100Hz -> 10ms
    scheduler.schedule(10000, [this](Time_us t){ this->sample(t); });
}

void MagEmulator::sample(Time_us current_time) {
    // 1. Propagate physics
    quadcopter.propagate_to(current_time);

    // 2. Get truth state
    SimState truth = quadcopter.getTruth();

    // 3. Get world model
    auto world_model = quadcopter.getDynamics()->getWorld();

    // 4. Get Ideal Measurement
    Vec3 mag_field_ned = world_model->getMagneticField(truth.pos_ned);
    Vec3 mag_field_body = truth.att.conjugate() * mag_field_ned;

    // 5. Add Noise
    Vec3 noise(mag_emu_dist(mag_emu_gen) * noise_std_, 
               mag_emu_dist(mag_emu_gen) * noise_std_, 
               mag_emu_dist(mag_emu_gen) * noise_std_);
    
    Vec3 noisy_mag = mag_field_body + noise + bias_;

    // 6. Get a buffer from DataManager
    SensorData* mag_data = m_sensor_buffer.claim();
    if (mag_data) {
        mag_data->sensor = SensorData::Type::MAG;
        mag_data->timestamp = current_time;
        mag_data->data.mag.mag[0] = noisy_mag.x();
        mag_data->data.mag.mag[1] = noisy_mag.y();
        mag_data->data.mag.mag[2] = noisy_mag.z();
        
        m_sensor_buffer.commit(mag_data);
    }

    // 7. Schedule next sample (100Hz -> 10000us)
    scheduler.schedule(10000, [this](Time_us t){ this->sample(t); });
}