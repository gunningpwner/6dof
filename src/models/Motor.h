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
    struct Config {
        // Physical Parameters (The "Truth" we want RLS to find)
        float time_constant = 0.050f;   // Tau (seconds) [cite: 297]
        float w_max = 5000.0f;          // Max rad/s
        float w_idle = 100.0f;          // Idle rad/s
        float nonlinearity = 0.5f;      // "Kappa" in paper [cite: 77]
                                        // 1.0 = Linear, 0.0 = Quadratic
        
        // Electrical (for battery sim later)
        float resistance = 0.150f;      // Ohms
        float kv = 2500.0f;             // RPM/Volt
    };

    Motor(const Config& cfg);

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
    Config cfg_;
    
    // Internal State
    float state_omega_ = 0.0f;      // rad/s
    float state_omega_dot_ = 0.0f;  // rad/s^2
    float current_draw_ = 0.0f;     // Amps
    float voltage_ = 0.0f;          // Volts
};