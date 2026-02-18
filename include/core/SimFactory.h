#pragma once

#include <memory>
#include "vehicle/Quadcopter.h"
#include "interfaces/IWorldModel.h"

class SimFactory {
public:
    // 1. Create a Standard Race Drone (Generic 5-inch, 4S battery)
    // You can pass in a specific world, or let it create a default Flat Earth one.
    static Quadcopter createStandardRaceDrone(std::shared_ptr<IWorldModel> world = nullptr);

    // 2. Create a Heavy Lift Drone (Larger props, lower KV, different inertia)
    static Quadcopter createHeavyLiftDrone(std::shared_ptr<IWorldModel> world = nullptr);

    // 3. Create a Custom Drone (Useful if you want to parameterize it from a config file later)
    static Quadcopter createCustomDrone(
        std::shared_ptr<IWorldModel> world,
        float mass, 
        Vec3 inertia, 
        float arm_length,
        float motor_kv
    );
};