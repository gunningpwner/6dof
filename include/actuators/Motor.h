#include "interfaces/IMotor.h"
#include "core/State.h"
class FirstOrderMotor : public IMotor {
    float tau = 0.025f;   // Tau (seconds) [cite: 297]
    float omega_max = 30000.0f;          // Max rad/s
    float omega_idle = 100.0f;          // Idle rad/s
    float kappa = 0.5f;      // "Kappa" in paper [cite: 77]
    
    double k_thrust_=1.3e-7;  // N / rpm^2
    double k_torque_=1.0e-8;  // Nm / rpm^2

    double cmd_ = 0.0;
    double rpm_ = 0.0;

public:
    FirstOrderMotor(){rpm_=omega_idle;};
    FirstOrderMotor(float tau, float omega_max, float omega_idle, float kappa, double k_thrust, double k_torque)
        : tau(tau), omega_max(omega_max), omega_idle(omega_idle), kappa(kappa), k_thrust_(k_thrust), k_torque_(k_torque) {rpm_=omega_idle;}

    void setCommand(double c) override { cmd_ = std::max(0.0, std::min(1.0, c)); }

    void update(double dt) override {
        float omega_s = omega_max*(kappa*cmd_ + (1-kappa)*sqrt(cmd_)) + omega_idle;
        float alpha =std::exp(-dt/tau);
        rpm_ =omega_s*(1-alpha) + rpm_*alpha;
    }

    MotorPhysicsOut getPhysics() const override {
        double thrust = k_thrust_ * rpm_ * rpm_;
        double torque = k_torque_ * rpm_ * rpm_;
        return {thrust, torque};
    }

    MotorTelemetry getTelemetry() const override {
        return {rpm_, 0.0, 0.0, 25.0};
    }
};