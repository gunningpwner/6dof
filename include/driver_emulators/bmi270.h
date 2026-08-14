#include "vehicle/Quadcopter.h"
#include "core/Scheduler.h"
#include "DataTypes.h"
class BMI270 {
public:
    BMI270(Quadcopter& quad, Scheduler& sched);

    bool processRawDataFAST(FastIMUData* data);

private:
    void sampleHardware(Time_us current_time); 
    Quadcopter& quadcopter;
    Scheduler& scheduler;
    FastIMUData latest_data;
};