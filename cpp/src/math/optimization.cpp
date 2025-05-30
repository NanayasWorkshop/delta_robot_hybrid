#include "math/optimization.hpp"
#include <algorithm>
#include <cmath>

namespace delta_robot {
namespace math {

Optimization::Optimization(double motor_limit, double min_height)
    : motor_limit_(motor_limit), min_height_(min_height) {
}

std::array<double, 3> Optimization::optimizeMotorPositions(const std::array<double, 3>& motor_positions) {
    // Store original positions for statistics
    last_stats_.original_positions = motor_positions;
    last_stats_.limits_exceeded = false;
    
    // Step 4a: Apply centering transformation
    std::array<double, 3> centered_positions = applyCentering(motor_positions);
    
    // Step 4b: Apply bounds constraints
    std::array<double, 3> final_positions = applyBoundsConstraints(centered_positions);
    
    // Store results for statistics
    last_stats_.optimized_positions = final_positions;
    
    // Check if any limits were exceeded
    for (const auto& pos : motor_positions) {
        if (std::abs(pos) > motor_limit_) {
            last_stats_.limits_exceeded = true;
            break;
        }
    }
    
    return final_positions;
}

bool Optimization::validateMotorLimits(const std::array<double, 3>& motor_positions) const {
    for (const auto& position : motor_positions) {
        if (std::abs(position) > motor_limit_) {
            return false;
        }
    }
    return true;
}

std::array<double, 3> Optimization::applyCentering(const std::array<double, 3>& positions) {
    // Calculate average
    double motor_avg = (positions[0] + positions[1] + positions[2]) / 3.0;
    
    // Store centering shift
    last_stats_.centering_shift = -motor_avg;
    
    // Apply centering (zero-mean transformation)
    return {
        positions[0] + last_stats_.centering_shift,
        positions[1] + last_stats_.centering_shift,
        positions[2] + last_stats_.centering_shift
    };
}

std::array<double, 3> Optimization::applyBoundsConstraints(const std::array<double, 3>& centered_positions) {
    // Find min and max of centered positions
    double min_centered = std::min({centered_positions[0], centered_positions[1], centered_positions[2]});
    double max_centered = std::max({centered_positions[0], centered_positions[1], centered_positions[2]});
    
    // Calculate bounds shift
    double bounds_shift = 0.0;
    
    if (min_centered < -motor_limit_) {
        bounds_shift = -motor_limit_ - min_centered;
    } else if (max_centered > motor_limit_) {
        bounds_shift = motor_limit_ - max_centered;
    }
    
    // Store bounds shift and calculate total shift
    last_stats_.bounds_shift = bounds_shift;
    last_stats_.total_shift = last_stats_.centering_shift + last_stats_.bounds_shift;
    
    // Apply bounds shift
    return {
        centered_positions[0] + bounds_shift,
        centered_positions[1] + bounds_shift,
        centered_positions[2] + bounds_shift
    };
}

} // namespace math
} // namespace delta_robot