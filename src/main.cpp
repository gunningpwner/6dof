#include <iostream>
#include <vector>
#include <iomanip>
#include "models/Motor.h"
#include <Eigen/Dense>
#include "KinematicAdjudicator.h"
#include "Logger.h"
#include "timing.h"

using Vector4f = Eigen::Vector4f;
using Vector3f = Eigen::Vector3f;
uint64_t g_current_time;
uint64_t getCurrentTimeUs()
{
    return g_current_time;
}
int main() {
    // 1. Setup
    Motor sim_motor;

    float dt = 0.0005f; // 1kHz simulation
    double sim_time = 0.0f;
    float battery_voltage = 16.0f; // 4S Lipo


    std::cout << "[Test] Running Motor Step Response..." << std::endl;

    KinematicAdjudicator autopilot;

    // 2. Simulation Loop (1 Second)
    while (sim_time < 1.0f) {
        
        // Generate a Step Input (0% -> 50% at 0.1s -> 0% at 0.6s)
        Vector4f omega_meas; 
        omega_meas << sim_motor.getOmega(), 0, 0, 0; // Populate all 4
        
        Vector3f acc_meas(0, 0, -9.81); // Simplified for now
        Vector3f gyro_meas(0, 0, 0);

        // C. Run the Autopilot (The Code Under Test)
        // Note: You need to pass the timestamp!
        g_current_time = (uint64_t)(sim_time * 1e6);
        Vector4f commands = autopilot.update(g_current_time, omega_meas, acc_meas, gyro_meas);
        Logger::getInstance().log("omega_raw", omega_meas, g_current_time);
        Logger::getInstance().log("command_raw", commands, g_current_time);
        // D. Apply Autopilot Commands to Sim Physics
        sim_motor.update(dt, commands(0), 16.0f);
        sim_time += dt;
    }

    return 0;
}