#include "TruthLogger.h"
#include "core/State.h"

TruthLogger::TruthLogger(Quadcopter& quad, Scheduler& scheduler, uint64_t interval_us)
    : quadcopter_(quad), scheduler_(scheduler), interval_us_(interval_us) {
    // Schedule the first log event.
    scheduler_.schedule(interval_us_, [this](Time_us t) { this->log(t); });
}

void TruthLogger::log(Time_us current_time) {
    // The scheduler gives us the current time.
    // We need to make sure the quadcopter state is up-to-date before logging.
    quadcopter_.propagate_to(current_time);

    // Log truth state
    SimState truth = quadcopter_.getTruth();
    Logger::getInstance().log("truth_pos_ned", truth.pos_ned, current_time);
    Logger::getInstance().log("truth_vel_ned", truth.vel_ned, current_time);
    // Eigen Quaternions are stored as (x, y, z, w) in coeffs()
    Logger::getInstance().log("truth_att_quat", truth.att.coeffs(), current_time);
    Logger::getInstance().log("truth_omega_body", truth.omega_body, current_time);
    Logger::getInstance().log("truth_accel_body", truth.accel_body, current_time);
    Logger::getInstance().log("truth_ang_acc_body", truth.ang_acc, current_time);

    // Log motor RPMs
    std::vector<double> rpms = quadcopter_.getMotorRPMs();
    Eigen::VectorXf rpm_vec(rpms.size());
    for(size_t i = 0; i < rpms.size(); ++i) {
        rpm_vec(i) = static_cast<float>(rpms[i]);
    }
    Logger::getInstance().log("truth_motor_rpms", rpm_vec, current_time);

    // Schedule the next log event
    scheduler_.schedule(interval_us_, [this](Time_us t) { this->log(t); });
}