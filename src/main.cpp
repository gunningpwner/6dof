#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>

// New Framework Includes
#include "vehicle/Quadcopter.h"
#include "world/FlatEarthModel.h"
#include "core/SimFactory.h"
#include "actuators/FirstOrderMotor.h" // We will create this below
#include "GeometricController.h"
#include "KinematicAdjudicator.h"
#include "Logger.h"
#include "DataManager.h"
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
    
    // B. The Vehicle
    Quadcopter quad= SimFactory::createPythonModelDrone(world);
    // -------------------------------------------------
    // 2. SETUP THE CONTROLLER
    // -------------------------------------------------
    std::cout << "[Test] Running Quadcopter Sim..." << std::endl;
    KinematicAdjudicator autopilot;
    


    DataManager::StateBuffer m_state_buffer;
    DataManager::StateConsumer m_state_consumer(m_state_buffer);

    GeometricController geometric_controller(m_state_consumer);
    float dt = 0.0005f; // 2kHz simulation
    double sim_time = 0.0f;
    Eigen::Quaternionf quat( 0.185f, -0.006f,  0.681f,  0.708f);
    quat.normalize(); 

    geometric_controller.rot_desired = quat.toRotationMatrix();


    // -------------------------------------------------
    // 3. SIMULATION LOOP
    // -------------------------------------------------
    while (sim_time < 14.0f) {
        g_current_time = (uint64_t)(sim_time * 1e6);

        std::vector<double> sim_rpms = quad.getMotorRPMs();
        
        Vector4f omega_meas;
        omega_meas << sim_rpms[0], sim_rpms[1], sim_rpms[2], sim_rpms[3];

        State truth = quad.getTruth();

        StateEstimate* est =m_state_buffer.claim();
        est->orientation = {truth.att.w(), truth.att.x(), truth.att.y(), truth.att.z()};
        est->position_enu = {0.0f, 0.0f, 0.0f};
        est->velocity_enu = { 0.0f, 0.0f, 0.0f};
        est->angular_vel = {truth.omega_body(0), truth.omega_body(1), truth.omega_body(2)};
        m_state_buffer.commit(est);

        geometric_controller.run();

        Logger::getInstance().log("Pos", truth.pos_ned, g_current_time);
        Logger::getInstance().log("Vel", truth.vel_ned, g_current_time);
        Vector4f quat;
        quat << truth.att.x(), truth.att.y(), truth.att.z(),truth.att.w();
        Logger::getInstance().log("Quat",quat , g_current_time);


        Vector3f acc_meas(0, 0, -9.81); 

        Vector3f gyro_meas= truth.omega_body;

        Logger::getInstance().log("Gyro", gyro_meas, g_current_time);
        Logger::getInstance().log("acc_cmd",geometric_controller.ang_acc_cmd , g_current_time);
        autopilot.setCommand(geometric_controller.ang_acc_cmd);


        Vector4f commands = autopilot.update(g_current_time, omega_meas, acc_meas, gyro_meas);
        
        
        Logger::getInstance().log("Control", commands, g_current_time);

        std::vector<double> cmd_vec = {commands(0), commands(1), commands(2), commands(3)};
        quad.setMotorCommands(cmd_vec);

        quad.step(dt);
        sim_time += dt;
    }

    return 0;
}