#pragma once

#include <memory>
#include "vehicle/Quadcopter.h"
#include "interfaces/IWorldModel.h"

class SimFactory {
public:
    // 1. Create a Standard Race Drone (Generic 5-inch, 4S battery)
    // You can pass in a specific world, or let it create a default Flat Earth one.
    static Quadcopter createPythonModelDrone(std::shared_ptr<IWorldModel> world = nullptr);

};