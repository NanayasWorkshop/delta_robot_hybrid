#pragma once

#include <array>
#include <vector>
#include <optional>
#include "calculation_result.hpp"
#include "math/math_orchestrator.hpp"
#include "utils/timing.hpp"

namespace delta_robot {

/**
 * @brief Main delta robot calculations class (legacy interface)
 * Now delegates to the new modular math orchestrator
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
     * @brief Main calculation function - delegates to orchestrator
     */
    std::optional<CalculationResult> calculateJointValues(const std::array<double, 3>& target_point);
    
    /**
     * @brief Legacy function that returns vector format for backward compatibility
     * @deprecated Use calculateJointValues() returning CalculationResult instead
     */
    std::optional<std::vector<double>> calculateJointValuesLegacy(const std::array<double, 3>& target_point);
    
    /**
     * @brief Get timing statistics from the last operation
     */
    utils::TimingStats getLastOperationStats() const;
    
    /**
     * @brief Get access to math orchestrator for advanced debugging
     */
    const math::MathOrchestrator& getMathOrchestrator() const { return orchestrator_; }

private:
    math::MathOrchestrator orchestrator_;
    
    /**
     * @brief Convert CalculationResult to legacy vector format
     */
    std::vector<double> resultToLegacyVector(const CalculationResult& result) const;
};

} // namespace delta_robot