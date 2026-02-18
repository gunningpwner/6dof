#include <Eigen/Dense>
#include "State.h"
using Vec3 = Eigen::Vector3f;

class IDynamics {
public:
    virtual ~IDynamics() = default;
    virtual void step(double dt, const Vec3& forces_body, const Vec3& torques_body) = 0;
    virtual State getState() const = 0;
    virtual void setState(const State& s) = 0;
};