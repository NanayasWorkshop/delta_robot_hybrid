#pragma once

#include <array>
#include <optional>
#include "math/kinematics.hpp"
#include "math/optimization.hpp"
#include "math/orientation.hpp"
#include "geometry/static_geometry.hpp"
#include "geometry/workspace.hpp"
#include "utils/timing.hpp"
#include "calculation_result.hpp"

namespace delta_robot {
namespace math {

/**
 * @brief Main orchestrator for delta robot mathematical calculations
 * Coordinates all mathematical modules in the correct sequence
 */
class MathOrchestrator {
public:
    /**
     * @brief Constructor with all required parameters
     */
    MathOrchestrator(
        double robot_radius,
        double min_height,
        double working_height,
        double motor_limit,
        double resting_position,
        double workspace_cone_angle_rad
    );
    
    /**
     * @brief Main calculation function - orchestrates all steps
     * @param target_point Target position [x, y, z]
     * @return Complete calculation result or nullopt if calculation fails
     */
    std::optional<CalculationResult> calculateJointValues(const std::array<double, 3>& target_point);
    
    /**
     * @brief Get detailed timing breakdown of last calculation
     */
    utils::TimingStats getLastOperationStats() const;
    
    /**
     * @brief Get access to individual module data for debugging/visualization
     */
    const Kinematics& getKinematics() const { return kinematics_; }
    const Optimization& getOptimization() const { return optimization_; }
    const Orientation& getOrientation() const { return orientation_; }
    const geometry::Workspace& getWorkspace() const { return workspace_; }

private:
    // Mathematical modules
    Kinematics kinematics_;
    Optimization optimization_;
    Orientation orientation_;
    
    // Geometry modules
    geometry::StaticGeometry static_geometry_;
    geometry::Workspace workspace_;
    
    // Timing tracking
    utils::TimingStats last_timing_stats_;
    
    /**
     * @brief Execute the complete calculation sequence
     */
    std::optional<CalculationResult> executeCalculationSequence(
        const std::array<double, 3>& corrected_target
    );
    
    /**
     * @brief Create final result structure
     */
    CalculationResult createResult(
        const std::array<double, 3>& original_target,
        const std::array<double, 3>& corrected_target,
        bool workspace_corrected,
        const std::array<double, 3>& optimized_motors,
        const std::array<double, 3>& fermat_point,
        double pitch,
        double roll,
        double prismatic_length,
        bool within_limits
    );
};

} // namespace math
} // namespace delta_robot