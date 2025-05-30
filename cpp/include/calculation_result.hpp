#pragma once

#include <array>

namespace delta_robot {

/**
 * @brief Result structure for delta robot calculations
 * Replaces the previous vector<double> approach with type-safe, named fields
 */
struct CalculationResult {
    // Joint angles and positions
    double pitch;                           // Roll angle in radians
    double roll;                           // Pitch angle in radians
    std::array<double, 3> motor_positions; // Motor positions [A, B, C]
    double prismatic_length;               // Prismatic actuator length
    
    // Geometric results
    std::array<double, 3> fermat_point;    // Fermat point coordinates [x, y, z]
    
    // Validation results
    bool within_limits;                    // True if all values are within operational limits
    
    // Target information
    std::array<double, 3> original_target;  // Original input target [x, y, z]
    std::array<double, 3> corrected_target; // Workspace-corrected target [x, y, z]
    bool workspace_corrected;              // True if workspace correction was applied
    
    // Default constructor
    CalculationResult() 
        : pitch(0.0), roll(0.0), motor_positions{0.0, 0.0, 0.0}, prismatic_length(0.0),
          fermat_point{0.0, 0.0, 0.0}, within_limits(false),
          original_target{0.0, 0.0, 0.0}, corrected_target{0.0, 0.0, 0.0},
          workspace_corrected(false) {}
};

} // namespace delta_robot