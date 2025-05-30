#pragma once

#include <array>

namespace delta_robot {
namespace geometry {

/**
 * @brief Coordinate transformation utilities
 */
class CoordinateTransforms {
public:
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
    static std::array<double, 3> crossProduct(
        const std::array<double, 3>& a, 
        const std::array<double, 3>& b
    );
    
    /**
     * @brief Calculate magnitude of a 3D vector
     * @param vec The vector
     * @return Magnitude of the vector
     */
    static double magnitude(const std::array<double, 3>& vec);
    
    /**
     * @brief Normalize a 3D vector
     * @param vec The vector to normalize
     * @return Normalized vector (unit vector)
     */
    static std::array<double, 3> normalize(const std::array<double, 3>& vec);
};

} // namespace geometry
} // namespace delta_robot
