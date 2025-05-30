#pragma once

#include <array>
#include <vector>
#include <optional>
#include "geometry/static_geometry.hpp"
#include "geometry/workspace.hpp"
#include "geometry/coordinate_transforms.hpp"
#include "utils/timing.hpp"

namespace delta_robot {

/**
 * @brief Main delta robot calculations class
 * Orchestrates all the mathematical calculations for inverse kinematics
 */
class DeltaCalculations {
public:
    /**
     * @brief Constructor with configuration parameters
     */
    DeltaCalculations(
        double robot_radius,
        double min_height,
        double working_height,
        double motor_limit,
        double resting_position,
        double workspace_cone_angle_rad
    );

    /**
     * @brief Main calculation function for joint values
     * @param target_point Target position [x, y, z]
     * @return Optional vector containing all calculated values, or nullopt if calculation fails
     */
    std::optional<std::vector<double>> calculateJointValues(const std::array<double, 3>& target_point);
    
    /**
     * @brief Get timing statistics from the last operation
     * @return Timing statistics structure
     */
    utils::TimingStats getLastOperationStats() const;
    
private:
    // Configuration parameters
    double min_height_;
    double working_height_;
    double motor_limit_;
    double resting_position_;
    double epsilon_;
    
    // Component modules
    geometry::StaticGeometry static_geometry_;
    geometry::Workspace workspace_;
    
    // Timing statistics
    utils::TimingStats last_timing_stats_;
    
    /**
     * @brief Calculate top positions of the actuators
     * @param target_point The target position
     * @return Z-coordinates of the three actuator top positions [Az, Bz, Cz]
     */
    std::array<double, 3> calculateTopPositions(const std::array<double, 3>& target_point);
    
    /**
     * @brief Calculate Fermat point from three points in 3D space
     * @param A_Point First point
     * @param B_Point Second point  
     * @param C_Point Third point
     * @return Fermat point coordinates [x, y, z]
     */
    std::array<double, 3> calculateFermatFromPoints(
        const std::array<double, 3>& A_Point,
        const std::array<double, 3>& B_Point,
        const std::array<double, 3>& C_Point
    );
    
    /**
     * @brief Optimize motor positions to stay within limits
     * @param motor_positions Initial motor positions [A, B, C]
     * @return Optimized motor positions
     */
    std::array<double, 3> optimizeMotorPositions(const std::array<double, 3>& motor_positions);
    
    /**
     * @brief Validate that all values are within operational limits
     * @param pitch Roll angle
     * @param roll Pitch angle
     * @param motor_positions Motor positions [A, B, C]
     * @return True if all values are within limits
     */
    bool validateLimits(double pitch, double roll, const std::array<double, 3>& motor_positions);
};

} // namespace delta_robot
