#pragma once
#include <Eigen/Dense>
#include "core/State.h"

class IWorldModel {
public:
    virtual ~IWorldModel() = default;
    virtual Vec3 getGravity(const Vec3& pos_ned) const = 0;
    virtual GeodeticPos nedToLLA(const Vec3& pos_ned) const = 0;
    virtual Vec3 llaToNED(const GeodeticPos& lla) const = 0;
};