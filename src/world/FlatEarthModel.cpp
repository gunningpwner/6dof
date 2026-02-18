#include "world/FlatEarthModel.h"
#include <cmath> // Only needed here for cos()

FlatEarthModel::FlatEarthModel(double lat0, double lon0) 
    : origin_{lat0, lon0, 0.0} {}

Vec3 FlatEarthModel::getGravity(const Vec3& pos_ned) const {
    // Implementation: Simple constant gravity
    return Vec3(0, 0, 9.81);
}

GeodeticPos FlatEarthModel::nedToLLA(const Vec3& ned) const {
    // Implementation: Linear projection
    GeodeticPos pos;
    pos.lat_rad = origin_.lat_rad + (ned.x() / R_EARTH);
    pos.lon_rad = origin_.lon_rad + (ned.y() / (R_EARTH * std::cos(origin_.lat_rad)));
    pos.alt_m = origin_.alt_m - ned.z();
    return pos;
}

Vec3 FlatEarthModel::llaToNED(const GeodeticPos& lla) const {
    // Implementation: Inverse projection
    Vec3 ned;
    ned.x() = (lla.lat_rad - origin_.lat_rad) * R_EARTH;
    ned.y() = (lla.lon_rad - origin_.lon_rad) * (R_EARTH * std::cos(origin_.lat_rad));
    ned.z() = origin_.alt_m - lla.alt_m;
    return ned;
}