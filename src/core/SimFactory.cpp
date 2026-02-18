#include "core/SimFactory.h"
#include "world/FlatEarthModel.h"
#include "dynamics/QuadDynamics.h"
#include "actuators/FirstOrderMotor.h"
#include <cmath>

Quadcopter SimFactory::createStandardRaceDrone(std::shared_ptr<IWorldModel> world) {
    // 1. Handle World (Create default if none provided)
    if (!world) {
        world = std::make_shared<FlatEarthModel>(0.0, 0.0);
    }

    // 2. Define Physics Properties (Typical 5" Race Drone)
    float mass = 0.6f; // 600g
    // Inertia: Ix, Iy, Iz (approximate for X frame)
    Vec3 inertia(0.005f, 0.005f, 0.01f); 

    // 3. Create Dynamics
    auto dynamics = std::make_shared<QuadDynamics>(world, mass, inertia);

    // 4. Create Vehicle
    Quadcopter quad(dynamics);

    // 5. Create & Add Motors
    // Configuration: 5" Props, 2306 Motors, 2450KV, 4S Battery
    float arm_len = 0.17f; // 170mm arm length
    float angle_45 = 0.7071f * arm_len; // X/Y component for 45 deg arms

    // Create a prototype motor to clone
    // Tau=0.05s, MaxRPM=35000, Idle=500, Kappa=1.0 (Linear), KT=Calculated, KQ=Calculated
    // Note: thrust/torque coeffs are simplified approximations here
    auto motor_proto = std::make_shared<FirstOrderMotor>(
        0.05f,      // Time Constant (simulated lag)
        35000.0f,   // Max RPM (approx 2450KV * 14.8V)
        500.0f,     // Idle RPM
        1.0f,       // Kappa (Linear response for simplicity)
        2.2e-7,     // Thrust Coefficient (N/rpm^2)
        1.0e-9      // Torque Coefficient (Nm/rpm^2)
    );

    // Add 4 Motors (Standard Betaflight X configuration)
    // Motor 1: Rear Right (CCW) -> Position ( -x, -y )
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_proto), {-angle_45, -angle_45, 0});
    
    // Motor 2: Front Right (CW)  -> Position ( +x, -y )
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_proto), { angle_45, -angle_45, 0});
    
    // Motor 3: Rear Left (CW)    -> Position ( -x, +y )
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_proto), {-angle_45,  angle_45, 0});
    
    // Motor 4: Front Left (CCW)  -> Position ( +x, +y )
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_proto), { angle_45,  angle_45, 0});

    return quad;
}