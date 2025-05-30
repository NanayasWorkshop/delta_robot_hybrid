#pragma once

#include <array>

namespace delta_robot {
namespace geometry {

/**
 * @brief Workspace validation and correction for delta robots
 */
class Workspace {
public:
    /**
     * @brief Constructor with workspace parameters
     * @param working_height The working height reference point
     * @param workspace_cone_angle_rad Maximum cone angle in radians
     */
    Workspace(double working_height, double workspace_cone_angle_rad);
    
    /**
     * @brief Verify and correct a target point to be within workspace limits
     * @param target_point The desired target position [x, y, z]
     * @return Corrected target point that lies within the workspace
     */
    std::array<double, 3> verifyAndCorrectTarget(const std::array<double, 3>& target_point) const;
    
    /**
     * @brief Check if a point is within the workspace cone
     * @param target_point The point to check
     * @return True if the point is within workspace limits
     */
    bool isWithinWorkspace(const std::array<double, 3>& target_point) const;
    
    /**
     * @brief Get the workspace cone angle
     * @return Workspace cone angle in radians
     */
    double getWorkspaceConeAngle() const;
    
private:
    double working_height_;
    double workspace_cone_angle_rad_;
    double epsilon_;
    
    /**
     * @brief Project a point onto the workspace cone boundary
     * @param target_point Original target point
     * @param H_to_target Vector from working height point to target
     * @param H_to_target_distance Distance of the vector
     * @return Corrected point on the cone boundary
     */
    std::array<double, 3> projectOntoConeBoundary(
        const std::array<double, 3>& target_point,
        const std::array<double, 3>& H_to_target,
        double H_to_target_distance
    ) const;
};

} // namespace geometry
} // namespace delta_robot
