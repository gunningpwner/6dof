#pragma once
#include "Types.h"
#include <iostream>

class Integrator {
public:
    // The "Equation of Motion" function type
    // Inputs: Current State, Time
    // Output: Derivatives (Vel, Accel, Omega, Alpha)
    using DynamicsFunction = Derivatives(*)(const State&, float dt);

    static void step(State& state, float dt, DynamicsFunction dynamics) {
        // 1. Calculate Derivatives based on current state
        Derivatives d = dynamics(state, dt);

        // 2. Integration (Explicit Euler for "Hello World")
        // Pos += Vel * dt
        state.position += d.pos_dot * dt;
        
        // Vel += Acc * dt
        state.velocity += d.vel_dot * dt;

        // Angle += Rate * dt (Small angle approx for now)
        // (Proper quaternion integration requires exp map, adding later)
        Vector3 angle_delta = state.angular_velocity * dt;
        Quaternion q_delta(Eigen::AngleAxisf(angle_delta.norm(), angle_delta.normalized()));
        state.attitude = state.attitude * q_delta; 
        state.attitude.normalize();

        // Rate += Alpha * dt
        state.angular_velocity += d.rate_dot * dt;

        // 3. Update Time
        state.time += dt;
    }
};