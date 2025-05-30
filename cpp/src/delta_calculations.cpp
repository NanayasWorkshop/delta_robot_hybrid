#include "delta_calculations.hpp"
#include <cmath>
#include <algorithm>

namespace delta_robot {

DeltaCalculations::DeltaCalculations(
    double robot_radius,
    double min_height,
    double working_height,
    double motor_limit,
    double resting_position,
    double workspace_cone_angle_rad
) : min_height_(min_height),
    working_height_(working_height),
    motor_limit_(motor_limit),
    resting_position_(resting_position),
    epsilon_(1e-10),
    static_geometry_(robot_radius),
    workspace_(working_height, workspace_cone_angle_rad)
{
}

std::optional<std::vector<double>> DeltaCalculations::calculateJointValues(const std::array<double, 3>& target_point) {
    utils::Timer total_timer;
    
    // Step 1: Verify and correct target
    utils::Timer verify_timer;
    std::array<double, 3> corrected_target = workspace_.verifyAndCorrectTarget(target_point);
    last_timing_stats_.verify_and_correct_ms = verify_timer.elapsed_ms();
    
    // Step 2: Calculate top positions
    utils::Timer top_positions_timer;
    auto [Ar_z, Br_z, Cr_z] = calculateTopPositions(corrected_target);
    last_timing_stats_.calculate_top_positions_ms = top_positions_timer.elapsed_ms();
    
    // Step 3: Calculate Fermat point
    utils::Timer fermat_timer;
    const auto& base_positions = static_geometry_.getBasePositions();
    auto [Ar_x, Ar_y] = base_positions[0];
    auto [Br_x, Br_y] = base_positions[1];
    auto [Cr_x, Cr_y] = base_positions[2];
    
    // Note: We'll create the A_Point, B_Point, C_Point when we need them in optimization
    last_timing_stats_.calculate_fermat_ms = fermat_timer.elapsed_ms();
    
    // Step 4: Optimize motor positions
    utils::Timer optimization_timer;
    std::array<double, 3> motor_positions = {Ar_z, Br_z, Cr_z};
    std::array<double, 3> optimized_motors = optimizeMotorPositions(motor_positions);
    
    // Recalculate with optimized positions
    std::array<double, 3> A_optimized = {Ar_x, Ar_y, optimized_motors[0]};
    std::array<double, 3> B_optimized = {Br_x, Br_y, optimized_motors[1]};
    std::array<double, 3> C_optimized = {Cr_x, Cr_y, optimized_motors[2]};
    
    std::array<double, 3> fermat_optimized = calculateFermatFromPoints(A_optimized, B_optimized, C_optimized);
    double fermat_z_optimized = fermat_optimized[2];
    
    double current_total_length = 2 * fermat_z_optimized + resting_position_;
    double prismatic_length = current_total_length - resting_position_;
    
    // Calculate plane normal
    std::array<double, 3> vec_AB = {
        B_optimized[0] - A_optimized[0],
        B_optimized[1] - A_optimized[1],
        B_optimized[2] - A_optimized[2]
    };
    
    std::array<double, 3> vec_AC = {
        C_optimized[0] - A_optimized[0],
        C_optimized[1] - A_optimized[1],
        C_optimized[2] - A_optimized[2]
    };
    
    std::array<double, 3> plane_normal = geometry::CoordinateTransforms::crossProduct(vec_AB, vec_AC);
    double plane_normal_length = geometry::CoordinateTransforms::magnitude(plane_normal);
    
    if (plane_normal_length < epsilon_) {
        last_timing_stats_.optimization_ms = optimization_timer.elapsed_ms();
        last_timing_stats_.total_ms = total_timer.elapsed_ms();
        return std::nullopt;
    }
    
    std::array<double, 3> plane_normal_unit = geometry::CoordinateTransforms::normalize(plane_normal);
    
    if (plane_normal_unit[2] < 0) {
        plane_normal_unit[0] = -plane_normal_unit[0];
        plane_normal_unit[1] = -plane_normal_unit[1];
        plane_normal_unit[2] = -plane_normal_unit[2];
    }
    
    auto [pitch, roll] = geometry::CoordinateTransforms::vectorToRollPitch(
        plane_normal_unit[0], plane_normal_unit[1], plane_normal_unit[2]
    );
    
    bool within_limits = validateLimits(pitch, roll, optimized_motors);
    
    last_timing_stats_.optimization_ms = optimization_timer.elapsed_ms();
    last_timing_stats_.total_ms = total_timer.elapsed_ms();
    
    // Prepare result
    std::vector<double> result = {
        pitch,
        roll,
        optimized_motors[0],
        optimized_motors[1],
        optimized_motors[2],
        prismatic_length,
        fermat_optimized[0],
        fermat_optimized[1],
        fermat_optimized[2],
        static_cast<double>(within_limits),
        target_point[0],
        target_point[1],
        target_point[2],
        corrected_target[0],
        corrected_target[1],
        corrected_target[2],
        static_cast<double>(!std::equal(target_point.begin(), target_point.end(), corrected_target.begin()))
    };
    
    return result;
}

std::array<double, 3> DeltaCalculations::calculateTopPositions(const std::array<double, 3>& target_point) {
    double target_x = target_point[0];
    double target_y = target_point[1];
    double target_z = target_point[2];
    
    std::array<double, 3> H = {0, 0, working_height_};
    std::array<double, 3> direction_vector = {target_x, target_y, target_z};
    
    double direction_length = geometry::CoordinateTransforms::magnitude(direction_vector);
    
    if (direction_length < epsilon_) {
        return {min_height_, min_height_, min_height_};
    }
    
    std::array<double, 3> plane_normal = geometry::CoordinateTransforms::normalize(direction_vector);
    
    std::array<double, 3> plane_center = {
        direction_vector[0] / 2,
        direction_vector[1] / 2,
        direction_vector[2] / 2
    };
    
    std::array<double, 3> H_to_center = {
        H[0] - plane_center[0],
        H[1] - plane_center[1],
        H[2] - plane_center[2]
    };
    
    double projection_length = 
        H_to_center[0] * plane_normal[0] +
        H_to_center[1] * plane_normal[1] +
        H_to_center[2] * plane_normal[2];
    
    std::array<double, 3> G = {
        H[0] - 2 * projection_length * plane_normal[0],
        H[1] - 2 * projection_length * plane_normal[1],
        H[2] - 2 * projection_length * plane_normal[2]
    };
    
    std::array<double, 3> u = {
        G[0] - H[0],
        G[1] - H[1],
        G[2] - H[2]
    };
    
    double u_x = u[0];
    double u_y = u[1];
    double u_z = u[2];
    
    if (std::abs(u_z) < epsilon_) {
        return {min_height_, min_height_, min_height_};
    }
    
    const auto& base_positions = static_geometry_.getBasePositions();
    auto [Ar_x, Ar_y] = base_positions[0];
    auto [Br_x, Br_y] = base_positions[1];
    auto [Cr_x, Cr_y] = base_positions[2];
    
    double Ar_z = -((u_x * Ar_x + u_y * Ar_y) / u_z);
    double Br_z = -((u_x * Br_x + u_y * Br_y) / u_z);
    double Cr_z = -((u_x * Cr_x + u_y * Cr_y) / u_z);
    
    return {Ar_z, Br_z, Cr_z};
}

std::array<double, 3> DeltaCalculations::calculateFermatFromPoints(
    const std::array<double, 3>& A_Point,
    const std::array<double, 3>& B_Point,
    const std::array<double, 3>& C_Point
) {
    std::array<double, 3> AB = {
        B_Point[0] - A_Point[0],
        B_Point[1] - A_Point[1],
        B_Point[2] - A_Point[2]
    };
    
    std::array<double, 3> BC = {
        C_Point[0] - B_Point[0],
        C_Point[1] - B_Point[1],
        C_Point[2] - B_Point[2]
    };
    
    std::array<double, 3> CA = {
        A_Point[0] - C_Point[0],
        A_Point[1] - C_Point[1],
        A_Point[2] - C_Point[2]
    };
    
    // Calculate lengths
    double a = geometry::CoordinateTransforms::magnitude(BC);
    double b = geometry::CoordinateTransforms::magnitude(CA);
    double c = geometry::CoordinateTransforms::magnitude(AB);
    
    // Calculate dot products
    double CA_dot_AB = -CA[0]*AB[0] - CA[1]*AB[1] - CA[2]*AB[2];
    double AB_dot_BC = -AB[0]*BC[0] - AB[1]*BC[1] - AB[2]*BC[2];
    double BC_dot_CA = -BC[0]*CA[0] - BC[1]*CA[1] - BC[2]*CA[2];
    
    double CA_norm = geometry::CoordinateTransforms::magnitude(CA);
    double AB_norm = geometry::CoordinateTransforms::magnitude(AB);
    double BC_norm = geometry::CoordinateTransforms::magnitude(BC);
    
    // Calculate angles
    double Alpha = std::acos(std::clamp(CA_dot_AB / (CA_norm * AB_norm), -1.0, 1.0));
    double Beta = std::acos(std::clamp(AB_dot_BC / (AB_norm * BC_norm), -1.0, 1.0));
    double Gamma = std::acos(std::clamp(BC_dot_CA / (BC_norm * CA_norm), -1.0, 1.0));
    
    // Calculate lambdas
    double LambdaA = a / std::max(std::sin(Alpha + M_PI/3), epsilon_);
    double LambdaB = b / std::max(std::sin(Beta + M_PI/3), epsilon_);
    double LambdaC = c / std::max(std::sin(Gamma + M_PI/3), epsilon_);
    
    double total_lambda = LambdaA + LambdaB + LambdaC;
    double Fermat_x = (LambdaA * A_Point[0] + LambdaB * B_Point[0] + LambdaC * C_Point[0]) / total_lambda;
    double Fermat_y = (LambdaA * A_Point[1] + LambdaB * B_Point[1] + LambdaC * C_Point[1]) / total_lambda;
    double Fermat_z = (LambdaA * A_Point[2] + LambdaB * B_Point[2] + LambdaC * C_Point[2]) / total_lambda;
    
    return {Fermat_x, Fermat_y, Fermat_z};
}

std::array<double, 3> DeltaCalculations::optimizeMotorPositions(const std::array<double, 3>& motor_positions) {
    double motor_avg = (motor_positions[0] + motor_positions[1] + motor_positions[2]) / 3;
    double centering_shift = -motor_avg;
    
    double motor_A_centered = motor_positions[0] + centering_shift;
    double motor_B_centered = motor_positions[1] + centering_shift;
    double motor_C_centered = motor_positions[2] + centering_shift;
    
    double min_centered = std::min({motor_A_centered, motor_B_centered, motor_C_centered});
    double max_centered = std::max({motor_A_centered, motor_B_centered, motor_C_centered});
    
    double bounds_shift = 0.0;
    if (min_centered < -motor_limit_) {
        bounds_shift = -motor_limit_ - min_centered;
    } else if (max_centered > motor_limit_) {
        bounds_shift = motor_limit_ - max_centered;
    }
    
    double total_shift = centering_shift + bounds_shift;
    
    return {
        motor_positions[0] + total_shift,
        motor_positions[1] + total_shift,
        motor_positions[2] + total_shift
    };
}

bool DeltaCalculations::validateLimits(double pitch, double roll, const std::array<double, 3>& motor_positions) {
    double revolute_limit = workspace_.getWorkspaceConeAngle();
    
    bool pitch_ok = std::abs(pitch) <= revolute_limit;
    bool roll_ok = std::abs(roll) <= revolute_limit;
    bool motor_A_ok = std::abs(motor_positions[0]) <= motor_limit_;
    bool motor_B_ok = std::abs(motor_positions[1]) <= motor_limit_;
    bool motor_C_ok = std::abs(motor_positions[2]) <= motor_limit_;
    
    return pitch_ok && roll_ok && motor_A_ok && motor_B_ok && motor_C_ok;
}

utils::TimingStats DeltaCalculations::getLastOperationStats() const {
    return last_timing_stats_;
}

} // namespace delta_robot
