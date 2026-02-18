#pragma once
#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

using Vec3 = Eigen::Vector3f;
using Quaternion = Eigen::Quaternionf;

// The "Truth" State
struct State {
    Vec3 pos_ned;      // Position in North-East-Down (m)
    Vec3 vel_ned;      // Velocity in North-East-Down (m/s)
    Quaternion att;    // Attitude (Body to NED)
    Vec3 omega_body;   // Angular Velocity in Body Frame (rad/s)
    double timestamp;
};

// Geodetic Coordinates
struct GeodeticPos {
    double lat_rad;
    double lon_rad;
    double alt_m;
};

// Motor Data
struct MotorPhysicsOut {
    double thrust_N;
    double torque_Nm;
};

struct MotorTelemetry {
    double rpm;
    double voltage;
    double current_amps;
    double temp_c;
};