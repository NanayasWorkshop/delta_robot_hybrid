#include "math/math_orchestrator.hpp"
#include <algorithm>
#include <cmath>

namespace delta_robot {
namespace math {

/**
 * @brief Normalize target point to exactly 100mm distance from origin
 * @param target_point Input target point [x, y, z]
 * @return Normalized target point at exactly 100mm distance with positive Z
 */
static std::array<double, 3> normalizeTargetTo100mm(const std::array<double, 3>& target_point) {
    double x = target_point[0];
    double y = target_point[1];
    double z = target_point[2];
    
    // Calculate current magnitude
    double magnitude = std::sqrt(x*x + y*y + z*z);
    
    // Handle zero vector case
    if (magnitude < 1e-10) {
        return {0.0, 0.0, 100.0};
    }
    
    // Ensure Z is positive (flip vector if needed)
    if (z < 0) {
        x = -x;
        y = -y;
        z = -z;
    }
    
    // Normalize and scale to exactly 100mm
    double scale_factor = 100.0 / magnitude;
    return {
        x * scale_factor,
        y * scale_factor,
        z * scale_factor
    };
}

MathOrchestrator::MathOrchestrator(
    double robot_radius,
    double min_height,
    double working_height,
    double motor_limit,
    double resting_position,
    double workspace_cone_angle_rad
) : kinematics_(working_height, static_geometry_),
    optimization_(motor_limit, min_height),
    orientation_(resting_position, workspace_cone_angle_rad),
    static_geometry_(robot_radius),
    workspace_(working_height, workspace_cone_angle_rad)
{
}

std::optional<CalculationResult> MathOrchestrator::calculateJointValues(const std::array<double, 3>& target_point) {
    utils::Timer total_timer;
    
    CalculationResult result;
    result.original_target = target_point;
    
    // NEW: Normalize input target to exactly 100mm distance
    std::array<double, 3> normalized_target = normalizeTargetTo100mm(target_point);
    
    // Step 1: Verify and correct the NORMALIZED target using workspace module
    utils::Timer verify_timer;
    std::array<double, 3> corrected_target = workspace_.verifyAndCorrectTarget(normalized_target);
    result.corrected_target = corrected_target;
    
    // Check if workspace correction was needed on the normalized target
    result.workspace_corrected = !std::equal(normalized_target.begin(), normalized_target.end(), corrected_target.begin());
    last_timing_stats_.verify_and_correct_ms = verify_timer.elapsed_ms();
    
    // Execute the main calculation sequence with the corrected normalized target
    auto sequence_result = executeCalculationSequence(corrected_target);
    
    if (!sequence_result) {
        last_timing_stats_.total_ms = total_timer.elapsed_ms();
        return std::nullopt;
    }
    
    // Update the result with original and corrected targets
    sequence_result->original_target = target_point;          // Keep original unnormalized
    sequence_result->corrected_target = corrected_target;     // This is the normalized + corrected
    sequence_result->workspace_corrected = result.workspace_corrected;
    
    last_timing_stats_.total_ms = total_timer.elapsed_ms();
    
    return sequence_result;
}

utils::TimingStats MathOrchestrator::getLastOperationStats() const {
    return last_timing_stats_;
}

std::optional<CalculationResult> MathOrchestrator::executeCalculationSequence(
    const std::array<double, 3>& corrected_target
) {
    // Step 2: Calculate top positions using kinematics module
    utils::Timer top_positions_timer;
    auto actuator_heights = kinematics_.calculateTopPositions(corrected_target);
    last_timing_stats_.calculate_top_positions_ms = top_positions_timer.elapsed_ms();
    
    // Step 3: Create 3D actuator positions for Fermat calculation
    utils::Timer fermat_timer;
    const auto& base_positions = static_geometry_.getBasePositions();
    
    std::array<std::array<double, 3>, 3> actuator_3d_positions = {{
        {base_positions[0][0], base_positions[0][1], actuator_heights[0]},
        {base_positions[1][0], base_positions[1][1], actuator_heights[1]},
        {base_positions[2][0], base_positions[2][1], actuator_heights[2]}
    }};
    
    // Calculate initial Fermat point (for timing statistics)
    kinematics_.calculateFermatPoint(actuator_3d_positions);
    last_timing_stats_.calculate_fermat_ms = fermat_timer.elapsed_ms();
    
    // Step 4: Optimize motor positions using optimization module
    utils::Timer optimization_timer;
    auto optimized_motors = optimization_.optimizeMotorPositions(actuator_heights);
    
    // Recalculate actuator positions with optimized motors
    std::array<std::array<double, 3>, 3> optimized_actuator_positions = {{
        {base_positions[0][0], base_positions[0][1], optimized_motors[0]},
        {base_positions[1][0], base_positions[1][1], optimized_motors[1]},
        {base_positions[2][0], base_positions[2][1], optimized_motors[2]}
    }};
    
    // Recalculate Fermat point with optimized positions
    auto optimized_fermat = kinematics_.calculateFermatPoint(optimized_actuator_positions);
    last_timing_stats_.optimization_ms = optimization_timer.elapsed_ms();
    
    // Step 5: Calculate orientation using orientation module
    auto [pitch, roll] = orientation_.calculateOrientation(optimized_actuator_positions);
    
    // Step 6: Calculate prismatic length using orientation module
    double prismatic_length = orientation_.calculatePrismaticLength(optimized_fermat);
    
    // Validate all limits
    bool motor_limits_ok = optimization_.validateMotorLimits(optimized_motors);
    bool orientation_limits_ok = orientation_.validateOrientation(pitch, roll);
    bool within_limits = motor_limits_ok && orientation_limits_ok;
    
    // Create and return the final result
    return createResult(
        {}, // Will be filled in by caller
        corrected_target,
        false, // Will be filled in by caller
        optimized_motors,
        optimized_fermat,
        pitch,
        roll,
        prismatic_length,
        within_limits
    );
}

CalculationResult MathOrchestrator::createResult(
    const std::array<double, 3>& original_target,
    const std::array<double, 3>& corrected_target,
    bool workspace_corrected,
    const std::array<double, 3>& optimized_motors,
    const std::array<double, 3>& fermat_point,
    double pitch,
    double roll,
    double prismatic_length,
    bool within_limits
) {
    CalculationResult result;
    
    result.pitch = pitch;
    result.roll = roll;
    result.motor_positions = optimized_motors;
    result.prismatic_length = prismatic_length;
    result.fermat_point = fermat_point;
    result.within_limits = within_limits;
    result.original_target = original_target;
    result.corrected_target = corrected_target;
    result.workspace_corrected = workspace_corrected;
    
    return result;
}

} // namespace math
} // namespace delta_robot