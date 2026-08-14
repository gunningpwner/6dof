#include "world/WGS84Model.h"
#include <cmath>
#include "WMM.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

WGS84Model::WGS84Model(double lat0, double lon0) 
    : origin_{lat0, lon0, 0.0} {}

// Simple constant gravity for now
Vec3 WGS84Model::getGravity(const Vec3& pos_ned) const {
    (void)pos_ned;
    return Vec3(0, 0, 9.81f);
}

// Simple linear projection, same as FlatEarthModel
GeodeticPos WGS84Model::nedToLLA(const Vec3& ned) const {
    GeodeticPos pos;
    pos.lat_rad = origin_.lat_rad + (ned.x() / R_EARTH);
    pos.lon_rad = origin_.lon_rad + (ned.y() / (R_EARTH * std::cos(origin_.lat_rad)));
    pos.alt_m = origin_.alt_m - ned.z();
    return pos;
}

// Simple inverse projection, same as FlatEarthModel
Vec3 WGS84Model::llaToNED(const GeodeticPos& lla) const {
    Vec3 ned;
    ned.x() = static_cast<float>((lla.lat_rad - origin_.lat_rad) * R_EARTH);
    ned.y() = static_cast<float>((lla.lon_rad - origin_.lon_rad) * (R_EARTH * std::cos(origin_.lat_rad)));
    ned.z() = static_cast<float>(origin_.alt_m - lla.alt_m);
    return ned;
}

Vec3 WGS84Model::getMagneticField(const Vec3& pos_ned) const {
    GeodeticPos current_pos = nedToLLA(pos_ned);

    float inc, dec;
    calcIncAndDec(current_pos.lat_rad * 180.0 / M_PI, 
                  current_pos.lon_rad * 180.0 / M_PI, 
                  inc, dec);

    return Vec3(cosf(inc) * cosf(dec),      // North
                cosf(inc) * sinf(dec),      // East
                sinf(inc)).normalized();    // Down
}