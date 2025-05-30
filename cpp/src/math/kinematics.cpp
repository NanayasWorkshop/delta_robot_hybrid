#include "math/kinematics.hpp"
#include "geometry/coordinate_transforms.hpp"
#include <cmath>
#include <algorithm>

namespace delta_robot {
namespace math {

Kinematics::Kinematics(double working_height, const geometry::StaticGeometry& static_geometry)
    : working_height_(working_height), static_geometry_(static_geometry) {
}

std::array<double, 3> Kinematics::calculateTopPositions(const std::array<double, 3>& target_point) {
    // Clear previous data
    last_data_ = KinematicsData{};
    
    // Step 2a: Create direction vector
    last_data_.direction_vector = createDirectionVector(target_point);
    last_data_.H_point = {0, 0, working_height_};
    
    double direction_length = geometry::CoordinateTransforms::magnitude(last_data_.direction_vector);
    
    if (direction_length < constants::EPSILON) {
        std::array<double, 3> fallback = {constants::MIN_HEIGHT, constants::MIN_HEIGHT, constants::MIN_HEIGHT};
        return fallback;
    }
    
    // Step 2b: Calculate plane normal
    last_data_.plane_normal = calculatePlaneNormal(last_data_.direction_vector);
    
    // Step 2c: Calculate plane center
    last_data_.plane_center = {
        last_data_.direction_vector[0] / 2,
        last_data_.direction_vector[1] / 2,
        last_data_.direction_vector[2] / 2
    };
    
    // Step 2d: Mirror working height point
    last_data_.G_point = mirrorWorkingHeightPoint(last_data_.plane_center, last_data_.plane_normal);
    
    // Step 2e: Calculate u vector
    last_data_.u_vector = {
        last_data_.G_point[0] - last_data_.H_point[0],
        last_data_.G_point[1] - last_data_.H_point[1],
        last_data_.G_point[2] - last_data_.H_point[2]
    };
    
    double u_x = last_data_.u_vector[0];
    double u_y = last_data_.u_vector[1];
    double u_z = last_data_.u_vector[2];
    
    if (std::abs(u_z) < constants::EPSILON) {
        std::array<double, 3> fallback = {constants::MIN_HEIGHT, constants::MIN_HEIGHT, constants::MIN_HEIGHT};
        return fallback;
    }
    
    // Step 2f: Calculate actuator heights using line-plane intersection
    const auto& base_positions = static_geometry_.getBasePositions();
    auto [Ar_x, Ar_y] = base_positions[0];
    auto [Br_x, Br_y] = base_positions[1];
    auto [Cr_x, Cr_y] = base_positions[2];
    
    double Ar_z = -((u_x * Ar_x + u_y * Ar_y) / u_z);
    double Br_z = -((u_x * Br_x + u_y * Br_y) / u_z);
    double Cr_z = -((u_x * Cr_x + u_y * Cr_y) / u_z);
    
    return {Ar_z, Br_z, Cr_z};
}

std::array<double, 3> Kinematics::calculateFermatPoint(
    const std::array<std::array<double, 3>, 3>& actuator_positions
) {
    const auto& A_Point = actuator_positions[0];
    const auto& B_Point = actuator_positions[1];
    const auto& C_Point = actuator_positions[2];
    
    // Step 3a: Calculate triangle sides
    last_data_.triangle_sides = calculateTriangleSides(actuator_positions);
    
    // Step 3b: Calculate triangle angles
    last_data_.triangle_angles = calculateTriangleAngles(actuator_positions, last_data_.triangle_sides);
    
    // Step 3c: Calculate lambda weights
    last_data_.lambda_weights = calculateLambdaWeights(last_data_.triangle_sides, last_data_.triangle_angles);
    
    // Step 3d: Calculate weighted Fermat point
    double LambdaA = last_data_.lambda_weights[0];
    double LambdaB = last_data_.lambda_weights[1];
    double LambdaC = last_data_.lambda_weights[2];
    
    double total_lambda = LambdaA + LambdaB + LambdaC;
    
    if (total_lambda < constants::EPSILON) {
        // Fallback to centroid
        return {
            (A_Point[0] + B_Point[0] + C_Point[0]) / 3,
            (A_Point[1] + B_Point[1] + C_Point[1]) / 3,
            (A_Point[2] + B_Point[2] + C_Point[2]) / 3
        };
    }
    
    double Fermat_x = (LambdaA * A_Point[0] + LambdaB * B_Point[0] + LambdaC * C_Point[0]) / total_lambda;
    double Fermat_y = (LambdaA * A_Point[1] + LambdaB * B_Point[1] + LambdaC * C_Point[1]) / total_lambda;
    double Fermat_z = (LambdaA * A_Point[2] + LambdaB * B_Point[2] + LambdaC * C_Point[2]) / total_lambda;
    
    return {Fermat_x, Fermat_y, Fermat_z};
}

// Private helper methods
std::array<double, 3> Kinematics::createDirectionVector(const std::array<double, 3>& target_point) {
    return {target_point[0], target_point[1], target_point[2]};
}

std::array<double, 3> Kinematics::calculatePlaneNormal(const std::array<double, 3>& direction_vector) {
    return geometry::CoordinateTransforms::normalize(direction_vector);
}

std::array<double, 3> Kinematics::mirrorWorkingHeightPoint(
    const std::array<double, 3>& plane_center,
    const std::array<double, 3>& plane_normal
) {
    std::array<double, 3> H_to_center = {
        last_data_.H_point[0] - plane_center[0],
        last_data_.H_point[1] - plane_center[1],
        last_data_.H_point[2] - plane_center[2]
    };
    
    double projection_length = 
        H_to_center[0] * plane_normal[0] +
        H_to_center[1] * plane_normal[1] +
        H_to_center[2] * plane_normal[2];
    
    return {
        last_data_.H_point[0] - 2 * projection_length * plane_normal[0],
        last_data_.H_point[1] - 2 * projection_length * plane_normal[1],
        last_data_.H_point[2] - 2 * projection_length * plane_normal[2]
    };
}

std::array<double, 3> Kinematics::calculateTriangleSides(
    const std::array<std::array<double, 3>, 3>& points
) {
    const auto& A_Point = points[0];
    const auto& B_Point = points[1];
    const auto& C_Point = points[2];
    
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
    double a = geometry::CoordinateTransforms::magnitude(BC);  // Length opposite to A
    double b = geometry::CoordinateTransforms::magnitude(CA);  // Length opposite to B
    double c = geometry::CoordinateTransforms::magnitude(AB);  // Length opposite to C
    
    return {a, b, c};
}

std::array<double, 3> Kinematics::calculateTriangleAngles(
    const std::array<std::array<double, 3>, 3>& points,
    const std::array<double, 3>& /* sides */
) {
    const auto& A_Point = points[0];
    const auto& B_Point = points[1];
    const auto& C_Point = points[2];
    
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
    
    // Calculate dot products
    double CA_dot_AB = -CA[0]*AB[0] - CA[1]*AB[1] - CA[2]*AB[2];
    double AB_dot_BC = -AB[0]*BC[0] - AB[1]*BC[1] - AB[2]*BC[2];
    double BC_dot_CA = -BC[0]*CA[0] - BC[1]*CA[1] - BC[2]*CA[2];
    
    double CA_norm = geometry::CoordinateTransforms::magnitude(CA);
    double AB_norm = geometry::CoordinateTransforms::magnitude(AB);
    double BC_norm = geometry::CoordinateTransforms::magnitude(BC);
    
    // Calculate angles using constants for clamping
    double Alpha = std::acos(std::clamp(CA_dot_AB / (CA_norm * AB_norm), 
                                       constants::TRIG_CLAMP_MIN, constants::TRIG_CLAMP_MAX));
    double Beta = std::acos(std::clamp(AB_dot_BC / (AB_norm * BC_norm), 
                                      constants::TRIG_CLAMP_MIN, constants::TRIG_CLAMP_MAX));
    double Gamma = std::acos(std::clamp(BC_dot_CA / (BC_norm * CA_norm), 
                                       constants::TRIG_CLAMP_MIN, constants::TRIG_CLAMP_MAX));
    
    return {Alpha, Beta, Gamma};
}

std::array<double, 3> Kinematics::calculateLambdaWeights(
    const std::array<double, 3>& sides,
    const std::array<double, 3>& angles
) {
    double a = sides[0];
    double b = sides[1];
    double c = sides[2];
    
    double Alpha = angles[0];
    double Beta = angles[1];
    double Gamma = angles[2];
    
    // Calculate lambdas using constants
    double LambdaA = a / std::max(std::sin(Alpha + constants::FERMAT_ANGLE_OFFSET), constants::FERMAT_MIN_DENOMINATOR);
    double LambdaB = b / std::max(std::sin(Beta + constants::FERMAT_ANGLE_OFFSET), constants::FERMAT_MIN_DENOMINATOR);
    double LambdaC = c / std::max(std::sin(Gamma + constants::FERMAT_ANGLE_OFFSET), constants::FERMAT_MIN_DENOMINATOR);
    
    return {LambdaA, LambdaB, LambdaC};
}

} // namespace math
} // namespace delta_robot