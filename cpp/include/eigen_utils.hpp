#pragma once

#include <Eigen/Dense>
#include <array>

namespace delta_robot {
namespace eigen_utils {

/**
 * @brief Utility functions for converting between Eigen and std::array types
 * This maintains backward compatibility while leveraging Eigen's performance
 */

// =================== Conversion Functions ===================

/**
 * @brief Convert std::array to Eigen::Vector3d
 */
inline Eigen::Vector3d arrayToVector3d(const std::array<double, 3>& arr) {
    return Eigen::Map<const Eigen::Vector3d>(arr.data());
}

/**
 * @brief Convert Eigen::Vector3d to std::array
 */
inline std::array<double, 3> vector3dToArray(const Eigen::Vector3d& vec) {
    std::array<double, 3> result;
    Eigen::Map<Eigen::Vector3d>(result.data()) = vec;
    return result;
}

/**
 * @brief Convert std::array to Eigen::Vector2d
 */
inline Eigen::Vector2d arrayToVector2d(const std::array<double, 2>& arr) {
    return Eigen::Map<const Eigen::Vector2d>(arr.data());
}

/**
 * @brief Convert Eigen::Vector2d to std::array
 */
inline std::array<double, 2> vector2dToArray(const Eigen::Vector2d& vec) {
    std::array<double, 2> result;
    Eigen::Map<Eigen::Vector2d>(result.data()) = vec;
    return result;
}

// =================== Enhanced Mathematical Operations ===================

/**
 * @brief Safe cross product with numerical stability
 */
inline Eigen::Vector3d safeCrossProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    Eigen::Vector3d result = a.cross(b);
    
    // Check for near-parallel vectors
    double cross_magnitude = result.norm();
    double a_magnitude = a.norm();
    double b_magnitude = b.norm();
    
    // If vectors are nearly parallel, return a default up vector
    if (cross_magnitude < 1e-10 * a_magnitude * b_magnitude) {
        return Eigen::Vector3d::UnitZ();
    }
    
    return result;
}

/**
 * @brief Safe normalization with fallback
 */
inline Eigen::Vector3d safeNormalized(const Eigen::Vector3d& vec, const Eigen::Vector3d& fallback = Eigen::Vector3d::UnitZ()) {
    double magnitude = vec.norm();
    if (magnitude < 1e-10) {
        return fallback;
    }
    return vec / magnitude;
}

/**
 * @brief Safe angle calculation between vectors
 */
inline double safeAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    Eigen::Vector3d a_norm = safeNormalized(a);
    Eigen::Vector3d b_norm = safeNormalized(b);
    
    double dot_product = a_norm.dot(b_norm);
    dot_product = std::clamp(dot_product, -1.0 + 1e-15, 1.0 - 1e-15);
    
    return std::acos(dot_product);
}

/**
 * @brief Calculate triangle properties using Eigen
 */
struct TriangleProperties {
    std::array<double, 3> side_lengths;
    std::array<double, 3> angles;
    Eigen::Vector3d centroid;
    Eigen::Vector3d normal;
    double area;
    bool is_degenerate;
};

/**
 * @brief Comprehensive triangle analysis
 */
TriangleProperties analyzeTriangle(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C);

/**
 * @brief Convert vector to roll/pitch angles with numerical stability
 */
inline std::array<double, 2> vectorToRollPitch(const Eigen::Vector3d& vec) {
    Eigen::Vector3d normalized = safeNormalized(vec);
    
    double roll = -std::atan2(normalized.y(), normalized.z());
    double pitch = std::atan2(normalized.x(), normalized.z());
    
    return {pitch, roll};
}

/**
 * @brief Check if a matrix/calculation is well-conditioned
 */
inline bool isWellConditioned(const Eigen::Matrix3d& matrix, double threshold = 1e-12) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(matrix);
    auto singular_values = svd.singularValues();
    double condition_number = singular_values(0) / singular_values(2);
    return condition_number < 1.0 / threshold;
}

} // namespace eigen_utils
} // namespace delta_robot