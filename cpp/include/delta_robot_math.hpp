#pragma once

#include <array>
#include <vector>
#include <optional>
#include "delta_calculations.hpp"
#include "utils/timing.hpp"

namespace delta_robot {

/**
 * @brief Main Delta Robot Mathematics class
 * This is the main interface that orchestrates all delta robot calculations
 */
class DeltaRobotMath {
public:
    // Re-export the TimingStats for backward compatibility
    using TimingStats = utils::TimingStats;

    /**
     * @brief Constructor with configuration parameters
     */
    DeltaRobotMath(
        double robot_radius,
        double min_height,
        double working_height,
        double motor_limit,
        double resting_position,
        double workspace_cone_angle_rad
    );

    /**
     * @brief Main calculation function
     * @param target_point Target position [x, y, z]
     * @return Optional vector containing all calculated values
     */
    std::optional<std::vector<double>> calculateJointValues(const std::array<double, 3>& target_point);
    
    /**
     * @brief Helper function to verify and correct targets
     * @param target_point Target position to verify
     * @return Corrected target position
     */
    std::array<double, 3> verifyAndCorrectTarget(const std::array<double, 3>& target_point);
    
    /**
     * @brief Get performance metrics from last operation
     * @return Timing statistics structure
     */
    TimingStats getLastOperationStats() const;
    
private:
    // The actual calculation engine
    DeltaCalculations calculator_;
    
    // Store parameters needed for workspace verification
    double working_height_;
    double workspace_cone_angle_rad_;
};

} // namespace delta_robot
