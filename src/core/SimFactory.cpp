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

    // Motor Constants
    float k_thrust = 1.3e-7; // N/rpm^2
    float k_torque = 1.0e-8; // Nm/rpm^2

    // CW Motor: Reaction torque is CCW (Negative Z in FRD)
    auto motor_cw = std::make_shared<FirstOrderMotor>(
        0.025f, 30000.0f, 100.0f, 0.5f, k_thrust, -k_torque
    );

    // CCW Motor: Reaction torque is CW (Positive Z in FRD)
    auto motor_ccw = std::make_shared<FirstOrderMotor>(
        0.025f, 30000.0f, 100.0f, 0.5f, k_thrust, k_torque
    );

    // Add 4 Motors (Standard Betaflight X configuration mapped to FRD)
    // FRD: X=Front, Y=Right, Z=Down
    
    // Motor 1: Rear Left (CW)    -> Position ( -x, -y ) 
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_cw), {-arm_len, -arm_len, 0});
    
    // Motor 2: Rear Right (CCW) -> Position ( -x, +y ) 
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_ccw), { -arm_len, arm_len, 0});
    
    // Motor 3: Front Right (CW)  -> Position ( +x, +y )
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_cw), {arm_len,  arm_len, 0});
    
    // Motor 4: Front Left (CCW)  -> Position ( +x, -y )
    quad.addMotor(std::make_shared<FirstOrderMotor>(*motor_ccw), { arm_len,  -arm_len, 0});

    return quad;
}