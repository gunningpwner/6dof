#pragma once
#include <Eigen/Dense>
#include "core/State.h"
class IDynamics {
public:
    virtual ~IDynamics() = default;
    virtual void step(double dt, const Vec3& forces_body, const Vec3& torques_body) = 0;
    virtual State getState() const = 0;
    virtual void setState(const State& s) = 0;
};