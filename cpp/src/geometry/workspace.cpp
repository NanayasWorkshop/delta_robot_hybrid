#include "geometry/workspace.hpp"
#include <cmath>
#include <algorithm>

namespace delta_robot {
namespace geometry {

Workspace::Workspace(double working_height, double workspace_cone_angle_rad)
    : working_height_(working_height),
      workspace_cone_angle_rad_(workspace_cone_angle_rad),
      epsilon_(1e-10) {
}

std::array<double, 3> Workspace::verifyAndCorrectTarget(const std::array<double, 3>& target_point) const {
    double target_x = target_point[0];
    double target_y = target_point[1];
    double target_z = target_point[2];
    
    std::array<double, 3> point_H = {0, 0, working_height_};
    std::array<double, 3> H_to_target = {
        target_x - point_H[0],
        target_y - point_H[1],
        target_z - point_H[2]
    };
    
    // Calculate distance
    double H_to_target_distance = std::sqrt(
        H_to_target[0] * H_to_target[0] +
        H_to_target[1] * H_to_target[1] +
        H_to_target[2] * H_to_target[2]
    );
    
    if (H_to_target_distance < epsilon_) {
        return {point_H[0], point_H[1], point_H[2] + 1.0};
    }
    
    std::array<double, 3> z_axis = {0, 0, 1};
    double dot_product = H_to_target[0] * z_axis[0] + H_to_target[1] * z_axis[1] + H_to_target[2] * z_axis[2];
    double angle_from_z = std::acos(std::clamp(dot_product / H_to_target_distance, -1.0, 1.0));
    
    if (H_to_target[2] < 0 || angle_from_z > workspace_cone_angle_rad_) {
        return projectOntoConeBoundary(target_point, H_to_target, H_to_target_distance);
    }
    
    return target_point;
}

bool Workspace::isWithinWorkspace(const std::array<double, 3>& target_point) const {
    std::array<double, 3> corrected = verifyAndCorrectTarget(target_point);
    return (std::abs(corrected[0] - target_point[0]) < epsilon_ &&
            std::abs(corrected[1] - target_point[1]) < epsilon_ &&
            std::abs(corrected[2] - target_point[2]) < epsilon_);
}

double Workspace::getWorkspaceConeAngle() const {
    return workspace_cone_angle_rad_;
}

std::array<double, 3> Workspace::projectOntoConeBoundary(
    const std::array<double, 3>& target_point,
    const std::array<double, 3>& H_to_target,
    double H_to_target_distance
) const {
    std::array<double, 3> point_H = {0, 0, working_height_};
    
    double xy_distance_at_max_angle = H_to_target_distance * std::sin(workspace_cone_angle_rad_);
    double z_distance_at_max_angle = H_to_target_distance * std::cos(workspace_cone_angle_rad_);
    
    double current_xy_distance = std::sqrt(H_to_target[0] * H_to_target[0] + H_to_target[1] * H_to_target[1]);
    std::array<double, 2> corrected_xy;
    
    if (current_xy_distance < epsilon_) {
        corrected_xy = {xy_distance_at_max_angle, 0};
    } else {
        double scale_factor = xy_distance_at_max_angle / current_xy_distance;
        corrected_xy = {H_to_target[0] * scale_factor, H_to_target[1] * scale_factor};
    }
    
    std::array<double, 3> corrected_H_to_target = {
        corrected_xy[0], corrected_xy[1], z_distance_at_max_angle
    };
    
    return {
        point_H[0] + corrected_H_to_target[0],
        point_H[1] + corrected_H_to_target[1],
        point_H[2] + corrected_H_to_target[2]
    };
}

} // namespace geometry
} // namespace delta_robot
