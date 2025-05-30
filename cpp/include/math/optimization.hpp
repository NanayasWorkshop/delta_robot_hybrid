#pragma once

#include <array>
#include "delta_constants.hpp"

namespace delta_robot {
namespace math {

/**
 * @brief Motor position optimization to respect physical constraints
 * Handles Step 4: Ensuring all motor positions stay within limits
 */
class Optimization {
public:
    /**
     * @brief Constructor with optimization parameters
     */
    Optimization(double motor_limit, double min_height);
    
    /**
     * @brief Optimize motor positions to stay within limits (Step 4)
     * @param motor_positions Initial motor positions [A, B, C]
     * @return Optimized motor positions that respect constraints
     */
    std::array<double, 3> optimizeMotorPositions(const std::array<double, 3>& motor_positions);
    
    /**
     * @brief Validate that positions are within limits
     * @param motor_positions Motor positions to validate
     * @return True if all positions are within operational limits
     */
    bool validateMotorLimits(const std::array<double, 3>& motor_positions) const;
    
    /**
     * @brief Get optimization statistics from last operation
     */
    struct OptimizationStats {
        double centering_shift;
        double bounds_shift;
        double total_shift;
        bool limits_exceeded;
        std::array<double, 3> original_positions;
        std::array<double, 3> optimized_positions;
    };
    
    const OptimizationStats& getLastOptimizationStats() const { return last_stats_; }

private:
    double motor_limit_;
    double min_height_;
    OptimizationStats last_stats_;
    
    /**
     * @brief Apply centering transformation (zero-mean)
     */
    std::array<double, 3> applyCentering(const std::array<double, 3>& positions);
    
    /**
     * @brief Apply bounds constraints
     */
    std::array<double, 3> applyBoundsConstraints(const std::array<double, 3>& centered_positions);
};

} // namespace math
} // namespace delta_robot