#pragma once

#include <array>
#include <Eigen/Dense>

namespace delta_robot {
namespace geometry {

/**
 * @brief Coordinate transformation utilities with Eigen integration
 * Provides both array-based (legacy) and Eigen-based (optimized) interfaces
 */
class CoordinateTransforms {
public:
    // =================== Legacy Array-Based Interface ===================
    
    /**
     * @brief Convert a 3D vector to roll and pitch angles
     * @param x X component of the vector
     * @param y Y component of the vector  
     * @param z Z component of the vector
     * @return Array containing [pitch, roll] in radians
     */
    static std::array<double, 2> vectorToRollPitch(double x, double y, double z);
    
    /**
     * @brief Calculate cross product of two 3D vectors
     * @param a First vector
     * @param b Second vector
     * @return Cross product a × b
     */
    static inline std::array<double, 3> crossProduct(
        const std::array<double, 3>& a, 
        const std::array<double, 3>& b
    );
    
    /**
     * @brief Calculate magnitude of a 3D vector
     * @param vec The vector
     * @return Magnitude of the vector
     */
    static inline double magnitude(const std::array<double, 3>& vec);
    
    /**
     * @brief Normalize a 3D vector
     * @param vec The vector to normalize
     * @return Normalized vector (unit vector)
     */
    static inline std::array<double, 3> normalize(const std::array<double, 3>& vec);
    
    // =================== Eigen-Based Interface (Optimized) ===================
    
    /**
     * @brief Convert a 3D vector to roll and pitch angles (Eigen version)
     * @param vec Input vector
     * @return Array containing [pitch, roll] in radians
     */
    static std::array<double, 2> vectorToRollPitch(const Eigen::Vector3d& vec);
    
    /**
     * @brief Calculate cross product using Eigen (SIMD optimized)
     * @param a First vector
     * @param b Second vector
     * @return Cross product a × b
     */
    static inline Eigen::Vector3d crossProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    
    /**
     * @brief Calculate magnitude using Eigen (SIMD optimized)
     * @param vec The vector
     * @return Magnitude of the vector
     */
    static inline double magnitude(const Eigen::Vector3d& vec);
    
    /**
     * @brief Normalize a vector using Eigen (SIMD optimized)
     * @param vec The vector to normalize
     * @return Normalized vector (unit vector)
     */
    static inline Eigen::Vector3d normalize(const Eigen::Vector3d& vec);
    
    // =================== Safe Mathematical Operations ===================
    
    /**
     * @brief Safe cross product with degenerate case handling
     */
    static Eigen::Vector3d safeCrossProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    
    /**
     * @brief Safe normalization with fallback
     */
    static Eigen::Vector3d safeNormalize(const Eigen::Vector3d& vec, const Eigen::Vector3d& fallback = Eigen::Vector3d::UnitZ());
    
    /**
     * @brief Safe angle calculation between vectors
     */
    static double safeAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
};

// =================== Inline Implementations ===================

inline std::array<double, 3> CoordinateTransforms::crossProduct(
    const std::array<double, 3>& a, 
    const std::array<double, 3>& b
) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

inline double CoordinateTransforms::magnitude(const std::array<double, 3>& vec) {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

inline std::array<double, 3> CoordinateTransforms::normalize(const std::array<double, 3>& vec) {
    double mag = magnitude(vec);
    // Use constants for consistency
    if (mag < 1e-10) {  // constants::VECTOR_MAGNITUDE_TOLERANCE
        return {0, 0, 1};  // Return up vector instead of zero
    }
    return {vec[0] / mag, vec[1] / mag, vec[2] / mag};
}

inline Eigen::Vector3d CoordinateTransforms::crossProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return a.cross(b);  // Eigen handles SIMD optimization automatically
}

inline double CoordinateTransforms::magnitude(const Eigen::Vector3d& vec) {
    return vec.norm();  // Eigen handles SIMD optimization automatically
}

inline Eigen::Vector3d CoordinateTransforms::normalize(const Eigen::Vector3d& vec) {
    double mag = vec.norm();
    if (mag < 1e-10) {  // constants::VECTOR_MAGNITUDE_TOLERANCE
        return Eigen::Vector3d::UnitZ();  // Return up vector
    }
    return vec / mag;
}

} // namespace geometry
} // namespace delta_robot