#include "core/State.h"

class IMotor {
public:
    virtual ~IMotor() = default;
    virtual void setCommand(double cmd_0to1) = 0;
    virtual void update(double dt) = 0;
    virtual MotorPhysicsOut getPhysics() const = 0;
    virtual MotorTelemetry getTelemetry() const = 0;
};