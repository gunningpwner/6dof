#include "Motor.h"
#include <iostream>

Motor::Motor(const Config& cfg) : cfg_(cfg) {
    state_omega_ = cfg_.w_idle;
}

void Motor::update(float dt, float throttle, float battery_voltage) {
    voltage_ = battery_voltage;

    // 1. Calculate Steady-State Target Omega (The "Ref")
    // Paper Eq (2): u = [kappa*d + (1-kappa)*sqrt(d)]^2
    // But we need Omega, so we use Eq (14) logic:
    // w_ref = w_max * (kappa*d + (1-kappa)*sqrt(d)) + w_idle
    
    // Clamp throttle 0.0 to 1.0
    float d = (throttle < 0.0f) ? 0.0f : (throttle > 1.0f) ? 1.0f : throttle;
    
    float mix = cfg_.nonlinearity * d + (1.0f - cfg_.nonlinearity) * std::sqrt(d);
    float w_ref = cfg_.w_max * mix + cfg_.w_idle;

    // 2. First Order Lag Dynamics
    // Tau * w_dot = w_ref - w
    // w_dot = (w_ref - w) / Tau
    state_omega_dot_ = (w_ref - state_omega_) / cfg_.time_constant;

    // 3. Integration
    state_omega_ += state_omega_dot_ * dt;

    // 4. Electrical Model (Simplified)
    // Current = (Voltage - EMF) / R
    // EMF = Omega / Kv_in_rads_per_sec
    float kv_rads = cfg_.kv * 0.104719755f; // RPM to rad/s
    float emf = state_omega_ / kv_rads;
    current_draw_ = (battery_voltage - emf) / cfg_.resistance;
    if(current_draw_ < 0) current_draw_ = 0.1f; // ESCs don't regen usually
}

float Motor::getThrust() const {
    // T = k * w^2
    // We'll define k arbitrarily for now or add it to Config
    float k = 2.0e-6f; 
    return k * state_omega_ * state_omega_;
}

float Motor::getTorque() const {
    // Q = b * w^2 + Inertia * w_dot
    float b = 1.0e-8f; 
    return b * state_omega_ * state_omega_;
}

DShotTelemetry Motor::getTelemetry() const {
    DShotTelemetry t;
    t.rpm = state_omega_ * 9.54929658f; // rad/s to RPM
    t.voltage = voltage_;
    t.current = current_draw_;
    t.temperature = 45.0f + (current_draw_ * 0.5f); // Fake temp rise
    return t;
}