#pragma once

#include <cmath>
#include <algorithm>

namespace delta_robot {
namespace constants {

// =================== Robot Physical Constants ===================
constexpr double ROBOT_RADIUS = 24.8;
constexpr double MIN_HEIGHT = 101.0;
constexpr double WORKING_HEIGHT = 11.5;
constexpr double MOTOR_LIMIT = 11.0;
constexpr double WORKSPACE_CONE_ANGLE_RAD = 0.5236;  // 30 degrees

// =================== Mathematical Constants ===================
constexpr double EPSILON = 1e-10;
constexpr double MS_CONVERSION_FACTOR = 1000.0;

// =================== Tolerance Constants ===================
// Different precision requirements for different types of calculations
constexpr double COORDINATE_TOLERANCE = 1e-3;        // 0.001 - for coordinate comparisons (mm precision)
constexpr double GEOMETRIC_TOLERANCE = 1e-6;         // For geometric calculations
constexpr double VECTOR_MAGNITUDE_TOLERANCE = 1e-10; // For vector normalization
constexpr double ANGLE_TOLERANCE = 1e-8;             // For angle calculations
constexpr double MIN_DISTANCE_TOLERANCE = 1e-6;      // Minimum meaningful distance
constexpr double WORKSPACE_DISTANCE_TOLERANCE = 1e-3; // Workspace boundary precision

// =================== NEW: Numerical Stability Constants ===================
constexpr double DEGENERATE_TRIANGLE_TOLERANCE = 1e-8;  // For nearly flat triangles
constexpr double PARALLEL_VECTOR_TOLERANCE = 1e-10;     // For parallel vectors
constexpr double SAFE_DIVISION_THRESHOLD = 1e-12;       // Safe division threshold
constexpr double NORMALIZED_DOT_EPSILON = 1e-15;        // For acos input safety
constexpr double MATRIX_CONDITION_THRESHOLD = 1e-12;    // For matrix conditioning

// =================== Trigonometric Constants ===================
// Clamping bounds for trigonometric functions to avoid domain errors
constexpr double TRIG_CLAMP_MIN = -1.0;
constexpr double TRIG_CLAMP_MAX = 1.0;
// Safe clamping with epsilon margins
constexpr double SAFE_TRIG_CLAMP_MIN = -1.0 + NORMALIZED_DOT_EPSILON;
constexpr double SAFE_TRIG_CLAMP_MAX = 1.0 - NORMALIZED_DOT_EPSILON;

// =================== Fermat Point Constants ===================
constexpr double FERMAT_ANGLE_OFFSET = M_PI / 3.0;  // 60 degrees for Fermat geometry
constexpr double FERMAT_MIN_DENOMINATOR = EPSILON;  // Prevent division by zero

// =================== Geometry Constants ===================
// Base actuator positions (angles in radians)
constexpr double BASE_A_ANGLE = M_PI / 2.0;                // 90 degrees (top)
constexpr double BASE_B_ANGLE = -M_PI / 6.0;        // -30 degrees (bottom right)
constexpr double BASE_C_ANGLE = -5.0 * M_PI / 6.0;  // -150 degrees (bottom left)

// =================== Utility Functions ===================
// Inline utility functions for better numerical stability
inline constexpr double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

inline constexpr double safe_clamp_trig(double value) {
    return clamp(value, SAFE_TRIG_CLAMP_MIN, SAFE_TRIG_CLAMP_MAX);
}

inline constexpr bool is_safe_for_division(double denominator, double threshold = SAFE_DIVISION_THRESHOLD) {
    return std::abs(denominator) > threshold;
}

inline constexpr bool is_degenerate_magnitude(double magnitude, double threshold = VECTOR_MAGNITUDE_TOLERANCE) {
    return magnitude < threshold;
}

// =================== Result Vector Indices (Legacy - To Be Removed) ===================
// These will be removed once the result structure is fully implemented
namespace result_indices {
    constexpr int PITCH = 0;
    constexpr int ROLL = 1;
    constexpr int MOTOR_A = 2;
    constexpr int MOTOR_B = 3;
    constexpr int MOTOR_C = 4;
    constexpr int PRISMATIC_LENGTH = 5;
    constexpr int FERMAT_X = 6;
    constexpr int FERMAT_Y = 7;
    constexpr int FERMAT_Z = 8;
    constexpr int WITHIN_LIMITS = 9;
    constexpr int ORIGINAL_TARGET_X = 10;
    constexpr int ORIGINAL_TARGET_Y = 11;
    constexpr int ORIGINAL_TARGET_Z = 12;
    constexpr int CORRECTED_TARGET_X = 13;
    constexpr int CORRECTED_TARGET_Y = 14;
    constexpr int CORRECTED_TARGET_Z = 15;
    constexpr int WORKSPACE_CORRECTED = 16;
} // namespace result_indices

} // namespace constants
} // namespace delta_robot