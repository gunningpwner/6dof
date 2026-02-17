#include "Motor.h"
#include <iostream>


void Motor::update(float dt, float throttle, float battery_voltage) {
    voltage_ = battery_voltage;

    // 1. Calculate Steady-State Target Omega (The "Ref")
    // Paper Eq (2): u = [kappa*d + (1-kappa)*sqrt(d)]^2
    // But we need Omega, so we use Eq (14) logic:
    // w_ref = w_max * (kappa*d + (1-kappa)*sqrt(d)) + w_idle
    
    // Clamp throttle 0.0 to 1.0
    float d = (throttle < 0.0f) ? 0.0f : (throttle > 1.0f) ? 1.0f : throttle;
    float omega_s = omega_max*(kappa*d + (1-kappa)*sqrt(d)) + omega_idle;
    float alpha =std::exp(-dt/tau);
    state_omega_ =omega_s*(1-alpha) + state_omega_*alpha;
    state_omega_dot_ = (omega_s - state_omega_) / tau;

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
    t.rpm = state_omega_;
    t.voltage = voltage_;
    t.current = current_draw_;
    t.temperature = 45.0f + (current_draw_ * 0.5f); // Fake temp rise
    return t;
}