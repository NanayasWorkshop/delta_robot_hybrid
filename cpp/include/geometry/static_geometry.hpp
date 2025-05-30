#pragma once

#include <array>

namespace delta_robot {
namespace geometry {

/**
 * @brief Static geometry calculations that depend only on robot parameters
 * These are calculated once and cached for performance
 */
class StaticGeometry {
public:
    /**
     * @brief Constructor with robot radius
     * @param robot_radius The radius of the delta robot base
     */
    explicit StaticGeometry(double robot_radius);
    
    /**
     * @brief Get the cached base positions
     * @return Array of 3 base positions, each with [x, y] coordinates
     */
    const std::array<std::array<double, 2>, 3>& getBasePositions() const;
    
    /**
     * @brief Get individual base position by index
     * @param index Base index (0=A, 1=B, 2=C)
     * @return Base position [x, y]
     */
    const std::array<double, 2>& getBasePosition(int index) const;
    
    /**
     * @brief Get the robot radius
     * @return Robot radius used in calculations
     */
    double getRobotRadius() const;
    
private:
    double robot_radius_;
    std::array<std::array<double, 2>, 3> base_positions_;
    
    /**
     * @brief Calculate base positions from robot radius
     * This is called once in the constructor
     */
    void calculateBasePositions();
};

} // namespace geometry
} // namespace delta_robot
