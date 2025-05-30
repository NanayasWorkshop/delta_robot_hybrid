#include "geometry/coordinate_transforms.hpp"
#include "delta_constants.hpp"
#include <cmath>

namespace delta_robot {
namespace geometry {

std::array<double, 2> CoordinateTransforms::vectorToRollPitch(double x, double y, double z) {
    // Use consistent tolerance from constants
    if (std::abs(x) < constants::COORDINATE_TOLERANCE && 
        std::abs(y) < constants::COORDINATE_TOLERANCE && 
        std::abs(z) < constants::COORDINATE_TOLERANCE) {
        return {0, 0};
    }
    double roll = -std::atan2(y, z);
    double pitch = std::atan2(x, z);
    return {pitch, roll};
}

std::array<double, 3> CoordinateTransforms::crossProduct(
    const std::array<double, 3>& a, 
    const std::array<double, 3>& b
) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

double CoordinateTransforms::magnitude(const std::array<double, 3>& vec) {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

std::array<double, 3> CoordinateTransforms::normalize(const std::array<double, 3>& vec) {
    double mag = magnitude(vec);
    // Use vector magnitude tolerance from constants
    if (mag < constants::VECTOR_MAGNITUDE_TOLERANCE) {
        return {0, 0, 0};
    }
    return {vec[0] / mag, vec[1] / mag, vec[2] / mag};
}

} // namespace geometry
} // namespace delta_robot