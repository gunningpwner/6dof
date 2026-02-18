#pragma once
#include <vector>
#include <memory>
#include "interfaces/IDynamics.h"
#include "interfaces/IMotor.h"
#include "interfaces/ISensor.h"

class Quadcopter {
    std::shared_ptr<IDynamics> dynamics_;
    std::vector<std::shared_ptr<IMotor>> motors_;
    std::vector<Vec3> motor_positions_;
    
    // Store sensors as generic base pointers
    std::vector<std::shared_ptr<ISensorBase>> sensors_;

public:
    Quadcopter(std::shared_ptr<IDynamics> dyn);

    void addMotor(std::shared_ptr<IMotor> m, Vec3 pos);
    void addSensor(std::shared_ptr<ISensorBase> s);

    void step(double dt);

    // Getters for GNC
    const State getTruth() const;
    std::vector<double> getMotorRPMs() const;
    void setMotorCommands(const std::vector<double>& cmds);
};