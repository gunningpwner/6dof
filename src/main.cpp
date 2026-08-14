#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <cmath>
#include "vehicle/Quadcopter.h"
#include "world/WGS84Model.h"
#include "world/FlatEarthModel.h"
#include "core/SimFactory.h"
#include "actuators/FirstOrderMotor.h" // We will create this below
#include "Logger.h"
#include "MCE.h"
#include "DataManager.h"
#include "driver_emulators/DShot.h"
#include "driver_emulators/bmi270.h"
#include "driver_emulators/GPSEmulator.h"
#include "driver_emulators/MagEmulator.h"
#include "TruthLogger.h"

BMI270* g_imu_ptr;
DShot* g_dshot_ptr;
DataManager* g_data_manager_ptr;

// Note: Ensure timing.h is compatible or remove if not needed for the sim
// #include "timing.h" 
// #include <chrono>


uint64_t g_current_time = 0;
uint64_t getCurrentTimeUs()
{
    return g_current_time;
}
int main(int argc, char** argv) {
    double max_time = 10.0;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-t" && i + 1 < argc) {
            max_time = std::stod(argv[++i]);
        }
    }

    // -------------------------------------------------
    // 1. SETUP THE SIMULATION FRAMEWORK
    // -------------------------------------------------
    Scheduler scheduler;
    // A. World & Dynamics
    // 1.0 kg mass, generic inertia
    // Origin set to Boulder, CO in radians.
    auto world = std::make_shared<WGS84Model>(40.0150 * M_PI / 180.0, -105.2705 * M_PI / 180.0);
    
    // B. The Vehicle
    Quadcopter quad= SimFactory::createPythonModelDrone(world);

    // C. Truth Logger
    TruthLogger truth_logger(quad, scheduler, 10000); // Log at 100Hz (10000 us)

    // -------------------------------------------------
    // 2. SETUP THE CONTROLLER
    // -------------------------------------------------
    std::cout << "[Test] Running Quadcopter Sim..." << std::endl;
    DataManager data_manager;
    g_data_manager_ptr = &data_manager;

    DShot dshot(quad, scheduler);
    g_dshot_ptr = &dshot;

    BMI270 imu(quad, scheduler);
    g_imu_ptr = &imu;

    GPSEmulator gps_emu(quad, scheduler, data_manager.getSensorBuffer());
    MagEmulator mag_emu(quad, scheduler, data_manager.getSensorBuffer());

    MonolithicControlEntity mce;
    mce.init();
    std::function<void(Time_us)> flightLoop = [&](Time_us t) {
        g_current_time = t;
        mce.run();
        scheduler.schedule(1000, flightLoop);
    };
    // Create the Flight Software Loop (e.g., 8000Hz -> 125us)
    std::function<void(Time_us)> fastFlightLoop = [&](Time_us t) {
        g_current_time = t;
        mce.runFastLoop();
        scheduler.schedule(125, fastFlightLoop);
    };
    scheduler.schedule(0, flightLoop);
    scheduler.schedule(1, fastFlightLoop);
    // Let it rip
    scheduler.run_until(10000000);

    std::cout << "[Test] Finished " << std::endl;
    

    return 0;
}