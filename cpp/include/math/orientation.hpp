#pragma once

#include <array>
#include <Eigen/Dense>
#include "geometry/coordinate_transforms.hpp"
#include "delta_constants.hpp"

namespace delta_robot {
namespace math {

/**
 * @brief Platform orientation and prismatic length calculations
 * Handles Steps 5 and 6: Pitch/Roll angles and prismatic actuator length
 * Enhanced with Eigen integration for better numerical stability
 */
class Orientation {
public:
    /**
     * @brief Constructor with robot parameters
     */
    Orientation(double resting_position, double workspace_cone_angle_rad);
    
    /**
     * @brief Calculate platform orientation from actuator positions (Step 5)
     * @param actuator_positions Optimized actuator positions [[x,y,z], [x,y,z], [x,y,z]]
     * @return Array containing [pitch, roll] in radians
     */
    std::array<double, 2> calculateOrientation(
        const std::array<std::array<double, 3>, 3>& actuator_positions
    );
    
    /**
     * @brief Calculate prismatic actuator length (Step 6)
     * @param fermat_point Fermat point from optimized actuator positions
     * @return Prismatic length in mm
     */
    double calculatePrismaticLength(const std::array<double, 3>& fermat_point);
    
    /**
     * @brief Validate orientation is within workspace limits
     * @param pitch Pitch angle in radians
     * @param roll Roll angle in radians
     * @return True if orientation is within workspace cone limits
     */
    bool validateOrientation(double pitch, double roll) const;
    
    /**
     * @brief Get orientation calculation details
     */
    struct OrientationData {
        std::array<double, 3> plane_normal;
        std::array<double, 3> vector_AB;
        std::array<double, 3> vector_AC;
        double plane_normal_magnitude;
        double pitch;
        double roll;
        double prismatic_length;
        bool within_orientation_limits;
    };
    
    const OrientationData& getLastOrientationData() const { return last_data_; }

private:
    double resting_position_;
    double workspace_cone_angle_rad_;
    OrientationData last_data_;
    
    /**
     * @brief Calculate plane normal from three actuator positions
     * Enhanced with Eigen for better numerical stability
     */
    std::array<double, 3> calculatePlaneNormal(
        const std::array<std::array<double, 3>, 3>& actuator_positions
    );
    
    /**
     * @brief Ensure plane normal points upward (z > 0)
     */
    std::array<double, 3> normalizeUpward(const std::array<double, 3>& plane_normal);
};

} // namespace math
} // namespace delta_robot