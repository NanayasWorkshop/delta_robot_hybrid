#pragma once

#include <array>
#include <vector>
#include <optional>
#include "geometry/static_geometry.hpp"
#include "geometry/workspace.hpp"
#include "geometry/coordinate_transforms.hpp"
#include "utils/timing.hpp"

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

/**
 * @brief Main delta robot calculations class
 * Orchestrates all the mathematical calculations for inverse kinematics
 */
class DeltaCalculations {
public:
    /**
     * @brief Constructor with configuration parameters
     */
    DeltaCalculations(
        double robot_radius,
        double min_height,
        double working_height,
        double motor_limit,
        double resting_position,
        double workspace_cone_angle_rad
    );

    /**
     * @brief Main calculation function for joint values
     * @param target_point Target position [x, y, z]
     * @return Optional CalculationResult containing all calculated values, or nullopt if calculation fails
     */
    std::optional<CalculationResult> calculateJointValues(const std::array<double, 3>& target_point);
    
    /**
     * @brief Legacy function that returns vector format for backward compatibility
     * @param target_point Target position [x, y, z]
     * @return Optional vector containing all calculated values, or nullopt if calculation fails
     * @deprecated Use calculateJointValues() returning CalculationResult instead
     */
    std::optional<std::vector<double>> calculateJointValuesLegacy(const std::array<double, 3>& target_point);
    
    /**
     * @brief Get timing statistics from the last operation
     * @return Timing statistics structure
     */
    utils::TimingStats getLastOperationStats() const;
    
private:
    // Configuration parameters
    double min_height_;
    double working_height_;
    double motor_limit_;
    double resting_position_;
    double epsilon_;
    
    // Component modules
    geometry::StaticGeometry static_geometry_;
    geometry::Workspace workspace_;
    
    // Timing statistics
    utils::TimingStats last_timing_stats_;
    
    /**
     * @brief Calculate top positions of the actuators
     * @param target_point The target position
     * @return Z-coordinates of the three actuator top positions [Az, Bz, Cz]
     */
    std::array<double, 3> calculateTopPositions(const std::array<double, 3>& target_point);
    
    /**
     * @brief Calculate Fermat point from three points in 3D space
     * @param A_Point First point
     * @param B_Point Second point  
     * @param C_Point Third point
     * @return Fermat point coordinates [x, y, z]
     */
    std::array<double, 3> calculateFermatFromPoints(
        const std::array<double, 3>& A_Point,
        const std::array<double, 3>& B_Point,
        const std::array<double, 3>& C_Point
    );
    
    /**
     * @brief Optimize motor positions to stay within limits
     * @param motor_positions Initial motor positions [A, B, C]
     * @return Optimized motor positions
     */
    std::array<double, 3> optimizeMotorPositions(const std::array<double, 3>& motor_positions);
    
    /**
     * @brief Validate that all values are within operational limits
     * @param pitch Roll angle
     * @param roll Pitch angle
     * @param motor_positions Motor positions [A, B, C]
     * @return True if all values are within limits
     */
    bool validateLimits(double pitch, double roll, const std::array<double, 3>& motor_positions);
    
    /**
     * @brief Convert CalculationResult to legacy vector format
     * @param result The calculation result structure
     * @return Vector in the legacy format
     */
    std::vector<double> resultToLegacyVector(const CalculationResult& result) const;
};

} // namespace delta_robot