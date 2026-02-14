#pragma once
#include <Eigen/Dense>

using Vector3 = Eigen::Vector3f;
using Quaternion = Eigen::Quaternionf;

struct State {
    // Kinematics
    Vector3 position{0, 0, 0};    // Position in World Frame (NED)
    Vector3 velocity{0, 0, 0};    // Velocity in World Frame
    Quaternion attitude{1, 0, 0, 0}; // Rotation from Body to World
    Vector3 angular_velocity{0, 0, 0}; // Angular Velocity in Body Frame

    // Time
    double time = 0.0;
};

struct Derivatives {
    Vector3 pos_dot;
    Vector3 vel_dot;
    Quaternion att_dot; // Note: This is usually represented as quaternion rate
    Vector3 rate_dot;
};