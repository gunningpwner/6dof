#pragma once
#include "interfaces/ISensor.h"
#include "interfaces/IWorldModel.h"
#include <memory>

struct GPSData {
    double lat_rad, lon_rad, alt_m;
};

class GPSSensor : public ISensor<GPSData> {
    std::shared_ptr<IWorldModel> world_;
    
    // Noise parameters
    double noise_sigma_h_; // Horizontal noise
    double noise_sigma_v_; // Vertical noise

public:
    GPSSensor(std::shared_ptr<IWorldModel> w, double sigma_h = 2.0, double sigma_v = 5.0);
    
    GPSData read(const State& truth) override;
};