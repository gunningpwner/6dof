#include <iostream>
#include <vector>
#include <iomanip>
#include "models/Motor.h"

// Simple CSV Logger
struct LogPoint {
    float time;
    float throttle;
    float omega;
    float omega_dot;
    float current;
};

int main() {
    // 1. Setup
    Motor::Config motor_cfg;
    motor_cfg.time_constant = 0.030f; // 30ms lag
    motor_cfg.w_max = 2000.0f;        // 2000 rad/s top speed
    Motor test_motor(motor_cfg);

    float dt = 0.001f; // 1kHz simulation
    float sim_time = 0.0f;
    float battery_voltage = 16.0f; // 4S Lipo

    std::vector<LogPoint> log;

    std::cout << "[Test] Running Motor Step Response..." << std::endl;

    // 2. Simulation Loop (1 Second)
    while (sim_time < 1.0f) {
        
        // Generate a Step Input (0% -> 50% at 0.1s -> 0% at 0.6s)
        float throttle = 0.0f;
        if (sim_time > 0.1f && sim_time < 0.6f) {
            throttle = 0.5f;
        }

        // Update Physics
        test_motor.update(dt, throttle, battery_voltage);

        // Log Data
        log.push_back({
            sim_time, 
            throttle, 
            test_motor.getOmega(), 
            test_motor.getOmegaDot(),
            test_motor.getTelemetry().current
        });

        sim_time += dt;
    }

    // 3. Output CSV to Console (Pipe this to a file!)
    std::cout << "time,throttle,omega,omega_dot,current" << std::endl;
    for (const auto& p : log) {
        std::cout << std::fixed << std::setprecision(4)
                  << p.time << "," 
                  << p.throttle << "," 
                  << p.omega << "," 
                  << p.omega_dot << ","
                  << p.current << std::endl;
    }

    return 0;
}