#include "vehicle/Quadcopter.h"
#include "core/Scheduler.h"
#include "DataTypes.h"
#include <cstdint>


enum DriverState { DISARMED, ARMED};

class DShot {
public:
    DShot(Quadcopter& quad, Scheduler& sched) 
        : quadcopter(quad), scheduler(sched) {};

    void disarm(){armedState = DISARMED;};
    void arm(){armedState = ARMED;};
    void sendMotorThrottle(float cmds[4]);
    uint8_t processTelemetry(FastRPMData* data);
private:
    void dmaCompleteMock(Time_us current_time);
    Quadcopter& quadcopter;
    Scheduler& scheduler;
    FastRPMData latest_telemetry;
    bool telemetry_ready = false;
    DriverState armedState = DISARMED;
    
};