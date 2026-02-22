#include "dynamics/QuadDynamics.h"
#include <iostream>

QuadDynamics::QuadDynamics(std::shared_ptr<IWorldModel> world, float mass, Vec3 inertia)
    : world_(world), mass_kg_(mass), inertia_diag_(inertia) {
    
    // Pre-calculate inverse inertia
    // Eigen's cwiseInverse() is perfect for this
    inertia_inv_diag_ = inertia.cwiseInverse();

    // Initialize State
    state_.pos_ned = Vec3::Zero();
    state_.vel_ned = Vec3::Zero();
    state_.att = Quaternion::Identity(); // w=1, x=0, y=0, z=0
    state_.omega_body = Vec3::Zero();
    state_.timestamp = 0.0;
}

void QuadDynamics::step(double dt, const Vec3& forces_body, const Vec3& torques_body) {
    // ---------------------------------------------------------
    // 1. TRANSLATIONAL DYNAMICS
    // ---------------------------------------------------------
    
    // Get Gravity (NED Frame)
    Vec3 gravity_ned = world_->getGravity(state_.pos_ned);
    
    state_.accel_body = forces_body / mass_kg_;
    // Rotate forces to NED: F_ned = q * F_body
    // Eigen overloads operator* for Quaternion * Vector3
    Vec3 forces_ned = state_.att * forces_body;

    // F = ma -> a = F/m + g
    Vec3 acc_ned = (forces_ned / mass_kg_) + gravity_ned;
    
    // ---------------------------------------------------------
    // 2. ROTATIONAL DYNAMICS (Euler's Eq)
    // ---------------------------------------------------------
    // I * alpha + w x (I * w) = Torque
    
    // Calculate Angular Momentum (L = I * w)
    // Since Inertia is diagonal, we use element-wise product
    Vec3 L = inertia_diag_.cwiseProduct(state_.omega_body);

    // Gyroscopic term: w x L
    Vec3 gyro_term = state_.omega_body.cross(L);

    // Solve for alpha: alpha = I_inv * (Torque - Gyro)
    Vec3 torque_residual = torques_body - gyro_term;
    Vec3 alpha_body = inertia_inv_diag_.cwiseProduct(torque_residual);

    // ---------------------------------------------------------
    // 3. INTEGRATION
    // ---------------------------------------------------------
    integrate(dt, acc_ned, alpha_body);
}

void QuadDynamics::integrate(double dt, const Vec3& acc_ned, const Vec3& alpha_body) {
    // 1. Linear Integration (Euler)
    state_.vel_ned += acc_ned * dt;
    state_.pos_ned += state_.vel_ned * dt;

    // 2. Angular Velocity Integration
    state_.omega_body += alpha_body * dt;

    // 3. Attitude Integration (Quaternion derivative)
    // q_dot = 0.5 * q * w (where w is a pure quaternion [0, wx, wy, wz])
    
    // Create pure quaternion from omega
    Quaternion w_pure(0, state_.omega_body.x(), state_.omega_body.y(), state_.omega_body.z());
    
    // Calculate derivative
    // Note: Eigen quaternions multiply as q_L * q_R
    Quaternion q_dot = state_.att * w_pure; 
    
    // Apply update: q_new = q_old + 0.5 * q_dot * dt
    // We access coefficients directly or construct a generic delta
    state_.att.coeffs() += q_dot.coeffs() * (0.5f * dt);

    // IMPORTANT: Normalize to prevent numerical drift
    state_.att.normalize();

    state_.timestamp += dt;
}

State QuadDynamics::getState() const {
    return state_;
}

void QuadDynamics::setState(const State& s) {
    state_ = s;
}