#include "vehicle/Quadcopter.h"
#include "core/Scheduler.h"
Quadcopter::Quadcopter(std::shared_ptr<IDynamics> dyn) : dynamics_(dyn) {}

void Quadcopter::addMotor(std::shared_ptr<IMotor> m, Vec3 pos) {
    motors_.push_back(m);
    motor_positions_.push_back(pos);
}

void Quadcopter::addSensor(std::shared_ptr<ISensorBase> s) {
    sensors_.push_back(s);
}

void Quadcopter::propagate_to(uint64_t target_time) {
    const Time_us max_dt_us = 10000; 

    while (last_update_time < target_time) {
        // Take the maximum allowed step, or whatever remainder is left
        Time_us current_dt_us = std::min(max_dt_us, target_time - last_update_time);
        
        // Convert to floating-point seconds for the physics integration
        double dt_sec = current_dt_us / 1000000.0;
        
        // Step the actual equations of motion
        step(dt_sec);
        
        // Advance the local physics clock by the exact integer microsecond amount
        last_update_time += current_dt_us;
    }
}

void Quadcopter::step(double dt) {
    Vec3 force_accum(0,0,0);
    Vec3 torque_accum(0,0,0);

    // 1. Process Motors
    for(size_t i=0; i < motors_.size(); ++i) {
        motors_[i]->update(dt);
        MotorPhysicsOut out = motors_[i]->getPhysics();

        // Convert Thrust (Scalar) to Body Force (Vector)
        // Assumption: Motors push UP (Negative Z in NED)
        Vec3 f_motor(0, 0, -out.thrust_N); 
        
        force_accum = force_accum + f_motor;

        // Torque = r x F + Drag Torque
        // (Drag torque sign depends on motor rotation direction CW/CCW)
        // Simplified: assuming we handle CW/CCW signs in configuration
        Vec3 t_motor = motor_positions_[i].cross(f_motor);
        t_motor.z() += out.torque_Nm; 
        
        torque_accum = torque_accum + t_motor;
    }

    // 2. Step Dynamics
    dynamics_->step(dt, force_accum, torque_accum);

}
const SimState Quadcopter::getTruth() const {
    // Delegate to the dynamics engine
    return dynamics_->getState();
}

std::vector<double> Quadcopter::getMotorRPMs() const {
    std::vector<double> rpms;
    rpms.reserve(motors_.size()); // Pre-allocate memory for speed

    for (const auto& motor : motors_) {
        // Extract just the RPM from the full telemetry struct
        rpms.push_back(motor->getTelemetry().rpm);
    }
    
    return rpms;
}

void Quadcopter::setMotorCommands(const std::vector<double>& cmds) {
    // Safety: Only loop up to the smaller of the two sizes
    // This prevents a segfault if your controller sends 4 cmds but you only have 3 motors
    size_t count = std::min(motors_.size(), cmds.size());

    for (size_t i = 0; i < count; ++i) {
        motors_[i]->setCommand(cmds[i]);
    }
}
