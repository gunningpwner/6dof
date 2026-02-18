#include "actuators/FirstOrderMotor.h"

// Default Constructor
FirstOrderMotor::FirstOrderMotor() 
    : tau_(0.025f), omega_max_(30000.0f), omega_idle_(100.0f), kappa_(0.5f), 
      k_thrust_(1.3e-7), k_torque_(1.0e-8), rpm_(100.0f) {}

// Parameterized Constructor
FirstOrderMotor::FirstOrderMotor(float tau, float omega_max, float omega_idle, float kappa, double k_thrust, double k_torque)
    : tau_(tau), omega_max_(omega_max), omega_idle_(omega_idle), kappa_(kappa), 
      k_thrust_(k_thrust), k_torque_(k_torque), rpm_(omega_idle) {}

void FirstOrderMotor::setCommand(double c) {
    // Clamp 0.0 to 1.0
    cmd_ = std::max(0.0, std::min(1.0, c));
}

void FirstOrderMotor::update(double dt) {
    // 1. Calculate Steady State Target (Omega_s)
    // Mixing linear and sqrt response based on Kappa
    double omega_s = omega_max_ * (kappa_ * cmd_ + (1.0f - kappa_) * std::sqrt(cmd_)) + omega_idle_;
    
    // 2. Discrete Time Alpha Calculation for First Order Lag
    // alpha = exp(-dt/tau)
    double alpha = std::exp(-dt / tau_);
    
    // 3. Apply Filter
    rpm_ = omega_s * (1.0 - alpha) + rpm_ * alpha;
}

MotorPhysicsOut FirstOrderMotor::getPhysics() const {
    // Compute Thrust and Torque based on squared RPM
    double thrust = k_thrust_ * rpm_ * rpm_;
    double torque = k_torque_ * rpm_ * rpm_;
    
    return {thrust, torque};
}

MotorTelemetry FirstOrderMotor::getTelemetry() const {
    // Return the state for the Flight Controller to read
    return {rpm_, 16.0, 0.0, 25.0}; // 16V, 25C constant for now
}