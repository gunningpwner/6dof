#include "driver_emulators/bmi270.h"

BMI270::BMI270(Quadcopter& quad, Scheduler& sched) : quadcopter(quad), scheduler(sched) {
    // Kick off the continuous sampling loop (e.g., 1000Hz -> 1000us)
    scheduler.schedule(1000, [this](Time_us t){ this->sampleHardware(t); });
}

bool BMI270::processRawDataFAST(FastIMUData* data) {
    *data = latest_data;
    return true;
}

void BMI270::sampleHardware(Time_us current_time) {
    // 1. Propagate physics to EXACTLY right now
    quadcopter.propagate_to(current_time);

    // 2. Read truth state, add noise, apply scaling
    SimState truth = quadcopter.getTruth();
    latest_data.data[0] = truth.accel_body.x(); // + noise
    latest_data.data[1] = truth.accel_body.y(); // + noise
    latest_data.data[2] = truth.accel_body.z(); // + noise

    latest_data.data[3] = truth.ang_acc.x(); // + noise
    latest_data.data[4] = truth.ang_acc.y(); // + noise
    latest_data.data[5] = truth.ang_acc.z(); // + noise
    // ... fill out remainder ...

    // 3. Schedule the next sample
    scheduler.schedule(1000, [this](Time_us t){ this->sampleHardware(t); });
}