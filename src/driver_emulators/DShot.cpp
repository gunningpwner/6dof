#include "driver_emulators/DShot.h"

void DShot::sendMotorThrottle(float cmds[4]){
    std::vector<double> double_cmds(cmds, cmds + 4);
    quadcopter.setMotorCommands(double_cmds);

    // 2. Simulate the DMA transfer time (e.g., 22us for Bi-DShot frame)
    // We schedule a lambda that mimics the hardware DMA Complete Interrupt
    scheduler.schedule(22, [this](Time_us t){ this->dmaCompleteMock(t); });
}

uint8_t DShot::processTelemetry(FastRPMData* data){
    if (!telemetry_ready) return 0;
        
    *data = latest_telemetry;
    telemetry_ready = false;
    return 4; // Returning 4 valid motors
}

void DShot::dmaCompleteMock(Time_us current_time) {
    // The "transfer" is done. Grab the real truth state from the plant.
    // Make sure the plant catches up to current_time first!
    quadcopter.propagate_to(current_time);
    
    std::vector<double> rpms = quadcopter.getMotorRPMs();
    for(int i = 0; i < 4; i++) {
        latest_telemetry.rpm[i] = rpms[i];
    }
    latest_telemetry.timestamp = current_time;
    telemetry_ready = true;
}