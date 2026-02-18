#pragma once
#include "interfaces/IWorldModel.h"

// Inherits from IWorldModel
class FlatEarthModel : public IWorldModel {
private:
    GeodeticPos origin_; // The "0,0,0" point
    const double R_EARTH = 6371000.0;

public:
    FlatEarthModel(double lat0, double lon0);

    // Overrides
    Vec3 getGravity(const Vec3& pos_ned) const override;
    GeodeticPos nedToLLA(const Vec3& pos_ned) const override;
    Vec3 llaToNED(const GeodeticPos& lla) const override;
};