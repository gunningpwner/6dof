#pragma once
#include <Eigen/Dense>
#include "core/State.h"
#include "interfaces/IWorldModel.h"
class IDynamics {
public:
    virtual ~IDynamics() = default;
    virtual void step(double dt, const Vec3& forces_body, const Vec3& torques_body) = 0;
    virtual SimState getState() const = 0;
    virtual void setState(const SimState& s) = 0;
    virtual std::shared_ptr<IWorldModel> getWorld() const = 0;
};