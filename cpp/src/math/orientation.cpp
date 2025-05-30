#include "math/orientation.hpp"
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
    
    // Step 5c: Convert normal vector to pitch and roll angles
    auto [pitch, roll] = geometry::CoordinateTransforms::vectorToRollPitch(
        last_data_.plane_normal[0], 
        last_data_.plane_normal[1], 
        last_data_.plane_normal[2]
    );
    
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
    const auto& A_optimized = actuator_positions[0];
    const auto& B_optimized = actuator_positions[1];
    const auto& C_optimized = actuator_positions[2];
    
    // Calculate vectors AB and AC
    last_data_.vector_AB = {
        B_optimized[0] - A_optimized[0],
        B_optimized[1] - A_optimized[1],
        B_optimized[2] - A_optimized[2]
    };
    
    last_data_.vector_AC = {
        C_optimized[0] - A_optimized[0],
        C_optimized[1] - A_optimized[1],
        C_optimized[2] - A_optimized[2]
    };
    
    // Calculate cross product to get plane normal
    std::array<double, 3> plane_normal = geometry::CoordinateTransforms::crossProduct(
        last_data_.vector_AB, last_data_.vector_AC
    );
    
    // Store magnitude for debugging
    last_data_.plane_normal_magnitude = geometry::CoordinateTransforms::magnitude(plane_normal);
    
    // Check if plane normal is valid (non-zero)
    if (last_data_.plane_normal_magnitude < constants::EPSILON) {
        // Return default upward normal if calculation fails
        return {0, 0, 1};
    }
    
    // Normalize the plane normal
    return geometry::CoordinateTransforms::normalize(plane_normal);
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