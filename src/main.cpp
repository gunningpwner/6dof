#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>

// New Framework Includes
#include "vehicle/Quadcopter.h"
#include "world/FlatEarthModel.h"
#include "dynamics/QuadDynamics.h"
#include "actuators/FirstOrderMotor.h" // We will create this below

#include "KinematicAdjudicator.h"
#include "Logger.h"

// Note: Ensure timing.h is compatible or remove if not needed for the sim
// #include "timing.h" 

using Vector4f = Eigen::Vector4f;
using Vector3f = Eigen::Vector3f;

uint64_t g_current_time = 0;
uint64_t getCurrentTimeUs()
{
    return g_current_time;
}
int main() {
    // -------------------------------------------------
    // 1. SETUP THE SIMULATION FRAMEWORK
    // -------------------------------------------------
    
    // A. World & Dynamics
    // 1.0 kg mass, generic inertia
    auto world = std::make_shared<FlatEarthModel>(0.0, 0.0);
    auto dynamics = std::make_shared<QuadDynamics>(world, 1.0, Vec3(0.01, 0.01, 0.02));
    
    // B. The Vehicle
    Quadcopter quad(dynamics);

    // C. Add Motors (Using your FirstOrderMotor)
    // Assuming X-Config: FL, FR, RL, RR (Order depends on your mixer!)
    // Positions: +/- 0.25m from center
    double arm = 0.25; 
    
    // Create a "prototype" motor with your specific params
    auto m_proto = std::make_shared<FirstOrderMotor>(
        0.025f,   // Tau
        30000.0f, // Max RPM (assuming RPM based on value)
        100.0f,   // Idle
        0.5f,     // Kappa
        1.3e-7,   // Thrust Coeff
        1.0e-8    // Torque Coeff
    );

    // Add 4 copies of this motor to the quad
    // (We use the copy constructor logic here effectively)
    quad.addMotor(std::make_shared<FirstOrderMotor>(*m_proto), { arm,  arm, 0}); // 0: FR
    quad.addMotor(std::make_shared<FirstOrderMotor>(*m_proto), {-arm,  arm, 0}); // 1: RL
    quad.addMotor(std::make_shared<FirstOrderMotor>(*m_proto), { arm, -arm, 0}); // 2: FL (Check your frame mapping)
    quad.addMotor(std::make_shared<FirstOrderMotor>(*m_proto), {-arm, -arm, 0}); // 3: RR

    // -------------------------------------------------
    // 2. SETUP THE CONTROLLER
    // -------------------------------------------------
    std::cout << "[Test] Running Quadcopter Sim..." << std::endl;
    KinematicAdjudicator autopilot;

    float dt = 0.0005f; // 2kHz simulation
    double sim_time = 0.0f;

    // -------------------------------------------------
    // 3. SIMULATION LOOP
    // -------------------------------------------------
    while (sim_time < 1.0f) {
        
        // --- A. READ SENSORS (From Sim Truth) ---
        // 1. Get RPMs from the Quad wrapper
        std::vector<double> sim_rpms = quad.getMotorRPMs();
        
        // Map std::vector to Eigen::Vector4f
        Vector4f omega_meas;
        omega_meas << sim_rpms[0], sim_rpms[1], sim_rpms[2], sim_rpms[3];

        // 2. Get IMU Data (Simplified for now, taking Truth)
        State truth = quad.getTruth();
        
        // Convert Sim Quaternion to whatever your Autopilot expects (Eigen?)
        // Vector3f acc_meas... (Ideally derived from truth.vel_ned derivative)
        Vector3f acc_meas(0, 0, -9.81); 
        // Vector3f gyro_meas = truth.omega_body;
        Vector3f gyro_meas(0, 0, 0);


        // --- B. RUN AUTOPILOT ---
        g_current_time = (uint64_t)(sim_time * 1e6);
        
        // The Autopilot decides what to do
        Vector4f commands = autopilot.update(g_current_time, omega_meas, acc_meas, gyro_meas);
        
        // Log Input/Output
        Logger::getInstance().log("omega_raw", omega_meas, g_current_time);
        Logger::getInstance().log("command_raw", commands, g_current_time);

        // --- C. APPLY COMMANDS TO SIM ---
        // Convert Eigen -> std::vector for the modular interface
        std::vector<double> cmd_vec = {commands(0), commands(1), commands(2), commands(3)};
        quad.setMotorCommands(cmd_vec);

        // --- D. STEP PHYSICS ---
        quad.step(dt);
        sim_time += dt;
    }

    return 0;
}