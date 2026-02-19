#include "core/SimFactory.h"
#include "world/FlatEarthModel.h"
#include "dynamics/QuadDynamics.h"
#include "actuators/FirstOrderMotor.h"
#include <cmath>

Quadcopter SimFactory::createPythonModelDrone(std::shared_ptr<IWorldModel> world) {
    // 1. Handle World (Create default if none provided)
    if (!world) {
        world = std::make_shared<FlatEarthModel>(0.0, 0.0);
    }

    // 2. Define Physics Properties (Typical 5" Race Drone)
    float mass = 1.0f; // 600g
    // Inertia: Ix, Iy, Iz (approximate for X frame)
    Vec3 inertia(0.1f, 0.1f, 0.2f); 

    // 3. Create Dynamics
    auto dynamics = std::make_shared<QuadDynamics>(world, mass, inertia);
    
    // 4. Create Vehicle
    Quadcopter quad(dynamics);

    // 5. Create & Add Motors
    // Configuration: 5" Props, 2306 Motors, 2450KV, 4S Battery
    float arm_len = 0.15f; // 170mm arm length

    // Create a prototype motor to clone
    // Tau=0.05s, MaxRPM=35000, Idle=500, Kappa=1.0 (Linear), KT=Calculated, KQ=Calculated
    // Note: thrust/torque coeffs are simplified approximations here
    auto motor_proto = std::make_shared<FirstOrderMotor>(
        0.025f,      // Time Constant (simulated lag)
        30000.0f,   // Max RPM (approx 2450KV * 14.8V)
        100.0f,     // Idle RPM
        0.5f,       // Kappa (Linear response for simplicity)
        1.3e-7,     // Thrust Coefficient (N/rpm^2)
        1.0e-8      // Torque Coefficient (Nm/rpm^2)
    );

    // Add 4 Motors (Standard Betaflight X configuration)
    // Motor 1: Rear Left (CW)    -> Position ( -x, +y ) 
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_proto), {-arm_len, arm_len, 0});
    
    // Motor 2: Rear Right (CCW) -> Position ( -x, -y ) 
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_proto), { -arm_len, -arm_len, 0});
    
    // Motor 3: Front Right (CW)  -> Position ( +x, -y )
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_proto), {arm_len,  -arm_len, 0});
    
    // Motor 4: Front Left (CCW)  -> Position ( +x, +y )
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_proto), { arm_len,  arm_len, 0});

    return quad;
}