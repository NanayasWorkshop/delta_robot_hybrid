#pragma once

#include <cmath>

namespace delta_robot {
namespace constants {

// Robot Physical Constants
constexpr double ROBOT_RADIUS = 24.8;
constexpr double MIN_HEIGHT = 101.0;
constexpr double WORKING_HEIGHT = 11.5;
constexpr double MOTOR_LIMIT = 11.0;
constexpr double WORKSPACE_CONE_ANGLE_RAD = 0.5236;  // 30 degrees

// Mathematical Constants
constexpr double EPSILON = 1e-10;
constexpr double MS_CONVERSION_FACTOR = 1000.0;

// Geometry Constants (Base actuator positions)
constexpr double BASE_A_ANGLE = 0.0;                // 0 degrees (top)
constexpr double BASE_B_ANGLE = -M_PI / 6.0;        // -30 degrees (bottom right)
constexpr double BASE_C_ANGLE = -5.0 * M_PI / 6.0;  // -150 degrees (bottom left)

} // namespace constants
} // namespace delta_robot
