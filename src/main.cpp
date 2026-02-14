#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include "Types.h"
#include "dynamics/Integrator.h"

// --- The Physics Model ---
// This function calculates F=ma. 
// Later, this will call your Motor, Aero, and Environment classes.
Derivatives rigid_body_dynamics(const State& state, float dt) {
    Derivatives d;

    // 1. Kinematics (Pos_dot = Vel)
    d.pos_dot = state.velocity;

    // 2. Forces (F = ma -> a = F/m)
    // For "Hello World", we just have Gravity pointing Down (Z+)
    Vector3 gravity(0, 0, 9.81f); 
    
    // Total Acceleration = Gravity + (Forces / Mass)
    d.vel_dot = gravity; 

    // 3. Rotational Dynamics (Euler's Eq: M = I*alpha + w x Iw)
    // For "Hello World", no torques.
    d.rate_dot = Vector3::Zero(); 

    return d;
}

int main() {
    std::cout << "[Sim] Initializing 6DOF Framework..." << std::endl;

    State vehicle_state;
    vehicle_state.position = Vector3(0, 0, -100); // Start 100m in the air (NED uses negative for Up)

    float dt = 0.01f; // 100 Hz
    float sim_duration = 5.0f;

    std::cout << "t(s) \t Pos Z (m) \t Vel Z (m/s)" << std::endl;
    std::cout << "-----------------------------------" << std::endl;

    // --- The Simulation Loop ---
    while (vehicle_state.time < sim_duration) {
        
        // 1. Step Physics
        Integrator::step(vehicle_state, dt, rigid_body_dynamics);

        // 2. Output Data (Simulating Telemetry)
        std::cout << std::fixed << std::setprecision(2) 
                  << vehicle_state.time << " \t " 
                  << vehicle_state.position.z() << " \t\t " 
                  << vehicle_state.velocity.z() << std::endl;

        // 3. Real-time pacing (Optional, usually we run sim as fast as possible)
        // std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt * 1000)));
    }

    std::cout << "[Sim] Simulation Complete." << std::endl;
    return 0;
}