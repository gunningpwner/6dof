#include "world/WGS84Model.h"
#include <cmath>

// WGS84 Ellipsoid Constants
const double a = 6378137.0; // Major axis
const double f = 1.0 / 298.257223563; // Flattening
const double e2 = 2*f - f*f; // Eccentricity squared

Vec3 WGS84Model::getGravity(const Vec3& pos_ned) const {
    // Implementation: Somigliana formula (Simplified)
    // Gravity varies with Latitude (stronger at poles)
    double sinLat = std::sin(origin_.lat_rad); // Approximation using origin lat
    double g = 9.780327 * (1 + 0.0053024 * sinLat*sinLat - 0.0000058 * std::sin(2*origin_.lat_rad)*std::sin(2*origin_.lat_rad));
    
    // Free Air Correction (Gravity decreases with altitude)
    // approx -3.086e-6 * altitude
    return Vec3(0, 0, g - (3.086e-6 * -pos_ned.z())); 
}

GeodeticPos WGS84Model::nedToLLA(const Vec3& ned) const {
    // Implementation: Local Tangent Plane (LTP) approximation
    // Suitable for short to medium ranges from origin
    GeodeticPos pos;

    // Radius of curvature in the prime vertical
    double sinLat = std::sin(origin_.lat_rad);
    double N = a / std::sqrt(1.0 - e2 * sinLat * sinLat);

    // Radius of curvature in the meridian
    double M = a * (1.0 - e2) / std::pow(1.0 - e2 * sinLat * sinLat, 1.5);

    pos.lat_rad = origin_.lat_rad + (ned.x() / M);
    pos.lon_rad = origin_.lon_rad + (ned.y() / (N * std::cos(origin_.lat_rad)));
    pos.alt_m = origin_.alt_m - ned.z();

    return pos;
}

Vec3 WGS84Model::llaToNED(const GeodeticPos& lla) const {
    // Implementation: Inverse Local Tangent Plane approximation
    double sinLat = std::sin(origin_.lat_rad);
    double N = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
    double M = a * (1.0 - e2) / std::pow(1.0 - e2 * sinLat * sinLat, 1.5);

    Vec3 ned;
    ned.x() = (lla.lat_rad - origin_.lat_rad) * M;
    ned.y() = (lla.lon_rad - origin_.lon_rad) * (N * std::cos(origin_.lat_rad));
    ned.z() = origin_.alt_m - lla.alt_m;

    return ned;
}

WGS84Model::WGS84Model(double lat0, double lon0) {
    origin_.lat_rad = lat0;
    origin_.lon_rad = lon0;
    origin_.alt_m = 0.0;
}