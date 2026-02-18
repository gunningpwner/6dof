#pragma once
#include "interfaces/IMotor.h"
#include <cmath>
#include <algorithm>

class FirstOrderMotor : public IMotor {
private:
    // Params
    float tau_;
    float omega_max_;
    float omega_idle_;
    float kappa_;
    double k_thrust_;
    double k_torque_;

    // State
    double cmd_ = 0.0;
    double rpm_; // Note: Storing current speed

public:
    // Default Constructor (Requested)
    FirstOrderMotor();

    // Parameterized Constructor
    FirstOrderMotor(float tau, float omega_max, float omega_idle, float kappa, double k_thrust, double k_torque);

    // --- IMotor Interface Implementation ---
    void setCommand(double c) override;
    void update(double dt) override;
    MotorPhysicsOut getPhysics() const override;
    MotorTelemetry getTelemetry() const override;
};