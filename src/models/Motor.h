#pragma once
#include <cmath>

struct DShotTelemetry {
    float rpm;          // Revolutions Per Minute
    float voltage;      // Volts
    float current;      // Amps
    float temperature;  // Celsius
};

class Motor {
public:

    Motor(){
        state_omega_=omega_idle;
    };
    Motor(float tau, float omega_max, float omega_idle, float kappa):
        tau(tau), omega_max(omega_max), omega_idle(omega_idle), kappa(kappa) {
            state_omega_=omega_idle;
    };

    // Run the physics for this timestep
    void update(float dt, float throttle_command_0_to_1, float battery_voltage);

    // Getters for Physics Engine
    float getThrust() const;
    float getTorque() const;
    float getOmega() const { return state_omega_; }
    float getOmegaDot() const { return state_omega_dot_; }

    // Getter for Autopilot (Sensor Simulation)
    DShotTelemetry getTelemetry() const;

private:

    float tau = 0.025f;   // Tau (seconds) [cite: 297]
    float omega_max = 30000.0f;          // Max rad/s
    float omega_idle = 100.0f;          // Idle rad/s
    float kappa = 0.5f;      // "Kappa" in paper [cite: 77]
    // Internal State
    float state_omega_ = 0.0f;      // rad/s
    float state_omega_dot_ = 0.0f;  // rad/s^2
    float current_draw_ = 0.0f;     // Amps
    float voltage_ = 0.0f;          // Volts
};