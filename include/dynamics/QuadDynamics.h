#pragma once

#include "interfaces/IDynamics.h"
#include "interfaces/IWorldModel.h"
#include "core/State.h" // Contains the 'using Vec3 = ...' aliases
#include <memory>

class QuadDynamics : public IDynamics {
private:
    std::shared_ptr<IWorldModel> world_;
    State state_;

    // Physical Properties
    float mass_kg_;
    Vec3 inertia_diag_;      // Diagonal Inertia (Ixx, Iyy, Izz)
    Vec3 inertia_inv_diag_;  // Pre-computed 1/I

    // Helper: Integration Step
    void integrate(double dt, const Vec3& acc_ned, const Vec3& alpha_body);

public:
    QuadDynamics(std::shared_ptr<IWorldModel> world, float mass, Vec3 inertia);
    virtual ~QuadDynamics() = default;

    void step(double dt, const Vec3& forces_body, const Vec3& torques_body) override;

    State getState() const override;
    void setState(const State& s) override;
};