#include "math/orientation.hpp"
#include "geometry/coordinate_transforms.hpp"
#include "eigen_utils.hpp"
#include <cmath>

namespace delta_robot {
namespace math {

Orientation::Orientation(double resting_position, double workspace_cone_angle_rad)
    : resting_position_(resting_position), workspace_cone_angle_rad_(workspace_cone_angle_rad) {
}

std::array<double, 2> Orientation::calculateOrientation(
    const std::array<std::array<double, 3>, 3>& actuator_positions
) {
    // Clear previous data
    last_data_ = OrientationData{};
    
    // Step 5a: Calculate plane normal from actuator positions
    last_data_.plane_normal = calculatePlaneNormal(actuator_positions);
    
    // Step 5b: Ensure plane normal points upward
    last_data_.plane_normal = normalizeUpward(last_data_.plane_normal);
    
    // Step 5c: Convert normal vector to pitch and roll angles using Eigen for better stability
    Eigen::Vector3d normal_eigen = eigen_utils::arrayToVector3d(last_data_.plane_normal);
    auto [pitch, roll] = geometry::CoordinateTransforms::vectorToRollPitch(normal_eigen);
    
    last_data_.pitch = pitch;
    last_data_.roll = roll;
    
    // Step 5d: Validate orientation limits
    last_data_.within_orientation_limits = validateOrientation(pitch, roll);
    
    return {pitch, roll};
}

double Orientation::calculatePrismaticLength(const std::array<double, 3>& fermat_point) {
    // Step 6: Calculate prismatic length from Fermat Z coordinate
    double fermat_z = fermat_point[2];
    double current_total_length = 2 * fermat_z + resting_position_;
    double prismatic_length = current_total_length - resting_position_;
    
    last_data_.prismatic_length = prismatic_length;
    
    return prismatic_length;
}

bool Orientation::validateOrientation(double pitch, double roll) const {
    return (std::abs(pitch) <= workspace_cone_angle_rad_) && 
           (std::abs(roll) <= workspace_cone_angle_rad_);
}

std::array<double, 3> Orientation::calculatePlaneNormal(
    const std::array<std::array<double, 3>, 3>& actuator_positions
) {
    // Use Eigen for better numerical stability and performance
    Eigen::Vector3d A = eigen_utils::arrayToVector3d(actuator_positions[0]);
    Eigen::Vector3d B = eigen_utils::arrayToVector3d(actuator_positions[1]);
    Eigen::Vector3d C = eigen_utils::arrayToVector3d(actuator_positions[2]);
    
    // Calculate vectors AB and AC
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AC = C - A;
    
    // Store for debugging (convert back to arrays)
    last_data_.vector_AB = eigen_utils::vector3dToArray(AB);
    last_data_.vector_AC = eigen_utils::vector3dToArray(AC);
    
    // Calculate cross product using safe method
    Eigen::Vector3d plane_normal = geometry::CoordinateTransforms::safeCrossProduct(AB, AC);
    
    // Store magnitude for debugging
    last_data_.plane_normal_magnitude = plane_normal.norm();
    
    // Check for degenerate triangles using improved detection
    if (constants::is_degenerate_magnitude(last_data_.plane_normal_magnitude)) {
        // Return default upward normal if calculation fails
        return {0, 0, 1};
    }
    
    // Additional check: ensure vectors aren't parallel using Eigen
    double cross_magnitude = plane_normal.norm();
    double ab_magnitude = AB.norm();
    double ac_magnitude = AC.norm();
    
    // Improved parallel vector detection
    if (cross_magnitude < constants::PARALLEL_VECTOR_TOLERANCE * ab_magnitude * ac_magnitude) {
        // Vectors are nearly parallel
        return {0, 0, 1};
    }
    
    // Normalize using safe method
    Eigen::Vector3d normalized = geometry::CoordinateTransforms::safeNormalize(plane_normal);
    
    return eigen_utils::vector3dToArray(normalized);
}

std::array<double, 3> Orientation::normalizeUpward(const std::array<double, 3>& plane_normal) {
    // Ensure the plane normal points upward (positive Z)
    if (plane_normal[2] < 0) {
        return {
            -plane_normal[0],
            -plane_normal[1], 
            -plane_normal[2]
        };
    }
    
    return plane_normal;
}

} // namespace math
} // namespace delta_robot