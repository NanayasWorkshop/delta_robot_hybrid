#include "geometry/coordinate_transforms.hpp"
#include "delta_constants.hpp"
#include "eigen_utils.hpp"
#include <cmath>

namespace delta_robot {
namespace geometry {

std::array<double, 2> CoordinateTransforms::vectorToRollPitch(double x, double y, double z) {
    // Improved numerical stability - normalize first
    double magnitude = std::sqrt(x*x + y*y + z*z);
    
    if (magnitude < constants::VECTOR_MAGNITUDE_TOLERANCE) {
        return {0, 0};
    }
    
    // Normalize for better numerical stability
    x /= magnitude;
    y /= magnitude; 
    z /= magnitude;
    
    double roll = -std::atan2(y, z);
    double pitch = std::atan2(x, z);
    return {pitch, roll};
}

std::array<double, 2> CoordinateTransforms::vectorToRollPitch(const Eigen::Vector3d& vec) {
    // Use Eigen version for better performance and stability
    return eigen_utils::vectorToRollPitch(vec);
}

Eigen::Vector3d CoordinateTransforms::safeCrossProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return eigen_utils::safeCrossProduct(a, b);
}

Eigen::Vector3d CoordinateTransforms::safeNormalize(const Eigen::Vector3d& vec, const Eigen::Vector3d& fallback) {
    return eigen_utils::safeNormalized(vec, fallback);
}

double CoordinateTransforms::safeAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return eigen_utils::safeAngleBetween(a, b);
}

} // namespace geometry
} // namespace delta_robot