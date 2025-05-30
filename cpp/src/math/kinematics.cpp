#include "math/kinematics.hpp"
#include "geometry/coordinate_transforms.hpp"
#include "eigen_utils.hpp"
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
    
    // Improved numerical stability check
    if (direction_length < constants::VECTOR_MAGNITUDE_TOLERANCE) {
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
    
    // Improved division by zero check using constants
    if (!constants::is_safe_for_division(u_z)) {
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
    // Use Eigen for improved numerical stability and performance
    Eigen::Vector3d A = eigen_utils::arrayToVector3d(actuator_positions[0]);
    Eigen::Vector3d B = eigen_utils::arrayToVector3d(actuator_positions[1]);
    Eigen::Vector3d C = eigen_utils::arrayToVector3d(actuator_positions[2]);
    
    // Analyze triangle properties using Eigen utilities
    auto triangle_props = eigen_utils::analyzeTriangle(A, B, C);
    
    // Store data for debugging
    last_data_.triangle_sides = triangle_props.side_lengths;
    last_data_.triangle_angles = triangle_props.angles;
    
    // Check for degenerate triangle
    if (triangle_props.is_degenerate) {
        // Return centroid for degenerate cases
        Eigen::Vector3d centroid = (A + B + C) / 3.0;
        return eigen_utils::vector3dToArray(centroid);
    }
    
    // Calculate lambda weights with improved numerical stability
    last_data_.lambda_weights = calculateLambdaWeights(last_data_.triangle_sides, last_data_.triangle_angles);
    
    // Calculate weighted Fermat point
    double LambdaA = last_data_.lambda_weights[0];
    double LambdaB = last_data_.lambda_weights[1];
    double LambdaC = last_data_.lambda_weights[2];
    
    double total_lambda = LambdaA + LambdaB + LambdaC;
    
    // Improved fallback handling
    if (!constants::is_safe_for_division(total_lambda)) {
        // Fallback to centroid
        Eigen::Vector3d centroid = (A + B + C) / 3.0;
        return eigen_utils::vector3dToArray(centroid);
    }
    
    // Calculate weighted average using Eigen
    Eigen::Vector3d fermat = (LambdaA * A + LambdaB * B + LambdaC * C) / total_lambda;
    
    return eigen_utils::vector3dToArray(fermat);
}

// Private helper methods
std::array<double, 3> Kinematics::createDirectionVector(const std::array<double, 3>& target_point) {
    return {target_point[0], target_point[1], target_point[2]};
}

std::array<double, 3> Kinematics::calculatePlaneNormal(const std::array<double, 3>& direction_vector) {
    // Use Eigen for better numerical stability
    Eigen::Vector3d dir = eigen_utils::arrayToVector3d(direction_vector);
    Eigen::Vector3d normalized = geometry::CoordinateTransforms::safeNormalize(dir);
    return eigen_utils::vector3dToArray(normalized);
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
    // Use Eigen for better numerical stability
    Eigen::Vector3d A = eigen_utils::arrayToVector3d(points[0]);
    Eigen::Vector3d B = eigen_utils::arrayToVector3d(points[1]);
    Eigen::Vector3d C = eigen_utils::arrayToVector3d(points[2]);
    
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d BC = C - B;
    Eigen::Vector3d CA = A - C;
    
    // Calculate angles using safe angle calculation
    double Alpha = geometry::CoordinateTransforms::safeAngleBetween(-CA, AB);
    double Beta = geometry::CoordinateTransforms::safeAngleBetween(-AB, BC);
    double Gamma = geometry::CoordinateTransforms::safeAngleBetween(-BC, CA);
    
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
    
    // Calculate lambdas with improved numerical stability
    auto safe_lambda = [](double side, double angle) -> double {
        double sin_value = std::sin(angle + constants::FERMAT_ANGLE_OFFSET);
        if (constants::is_safe_for_division(sin_value)) {
            return side / sin_value;
        }
        return 0.0;  // Safe fallback
    };
    
    double LambdaA = safe_lambda(a, Alpha);
    double LambdaB = safe_lambda(b, Beta);
    double LambdaC = safe_lambda(c, Gamma);
    
    return {LambdaA, LambdaB, LambdaC};
}

} // namespace math
} // namespace delta_robot