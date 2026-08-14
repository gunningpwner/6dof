#pragma once

#include "vehicle/Quadcopter.h"
#include "core/Scheduler.h"
#include "Logger.h"

class TruthLogger {
public:
    TruthLogger(Quadcopter& quad, Scheduler& scheduler, uint64_t interval_us = 10000); // 100Hz default

private:
    void log(Time_us current_time);

    Quadcopter& quadcopter_;
    Scheduler& scheduler_;
    uint64_t interval_us_;
};