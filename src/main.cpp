#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <string>

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
// #include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

using Vector4f = Eigen::Vector4f;
using Vector3f = Eigen::Vector3f;

uint64_t g_current_time = 0;
uint64_t getCurrentTimeUs()
{
    return g_current_time;
}
int main(int argc, char** argv) {
    double max_time = 10.0;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-t" && i + 1 < argc) {
            max_time = std::stod(argv[++i]);
        }
    }

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
    // Eigen::Quaternionf quat( 0.185f, -0.006f,  0.681f,  0.708f);
    // Eigen::Quaternionf quat( 1.0f, 0.0f,  0.0f,  0.0f);
    // quat.normalize(); 

    // geometric_controller.rot_desired = quat.toRotationMatrix();
    geometric_controller.vel_cmd = Vec3(0.0f, 0.0f, 0.0f);


    // -------------------------------------------------
    // 3. SIMULATION LOOP
    // -------------------------------------------------
    while (sim_time <= max_time) {
        // auto step_start = high_resolution_clock::now();
        g_current_time = (uint64_t)(sim_time * 1e6);
        if (g_current_time%10000000 == 0)
            printf("Sim Time: %f\n", sim_time);

        std::vector<double> sim_rpms = quad.getMotorRPMs();
        
        Vector4f omega_meas;
        omega_meas << sim_rpms[0], sim_rpms[1], sim_rpms[2], sim_rpms[3];

        State truth = quad.getTruth();

        StateEstimate* est =m_state_buffer.claim();
        est->orientation = {truth.att.x(), truth.att.y(), truth.att.z(),truth.att.w()};
        est->position_ned = {truth.pos_ned(0), truth.pos_ned(1), truth.pos_ned(2)};
        est->velocity_ned = {truth.vel_ned(0), truth.vel_ned(1), truth.vel_ned(2)};
        est->angular_vel = {truth.omega_body(0), truth.omega_body(1), truth.omega_body(2)};
        m_state_buffer.commit(est);

        geometric_controller.run();



        Vector4f quat;
        quat << truth.att.x(), truth.att.y(), truth.att.z(),truth.att.w();
        

        Vector3f acc_meas = truth.accel_body; 

        Vector3f gyro_meas= truth.omega_body;


        Vector3f ang_acc_cmd = geometric_controller.ang_acc_cmd;
        float lin_cmd = geometric_controller.linear_z_accel_cmd;

        
        
        autopilot.setCommand(ang_acc_cmd,lin_cmd);


        Vector4f commands = autopilot.update(g_current_time, omega_meas, acc_meas, gyro_meas);
  


        Logger::getInstance().log("Pos", truth.pos_ned, g_current_time);
        Logger::getInstance().log("Vel", truth.vel_ned, g_current_time);
        Logger::getInstance().log("Quat",quat , g_current_time);
        Logger::getInstance().log("Gyro", gyro_meas, g_current_time);
        Logger::getInstance().log("IMU", acc_meas, g_current_time);
        Logger::getInstance().log("acc_cmd",ang_acc_cmd , g_current_time);
        Logger::getInstance().log("lin_cmd",lin_cmd , g_current_time);
        Logger::getInstance().log("Control", commands, g_current_time);

        

        std::vector<double> cmd_vec = {commands(0), commands(1), commands(2), commands(3)};
        quad.setMotorCommands(cmd_vec);

        quad.step(dt);
        sim_time += dt;
        // auto step_stop = high_resolution_clock::now();
        // duration<double, std::milli> ms_double = step_stop - step_start;
        // printf("Step time: %f ms\n", ms_double.count());
    }

    return 0;
}