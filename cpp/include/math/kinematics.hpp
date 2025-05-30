#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include "geometry/static_geometry.hpp"
#include "delta_constants.hpp"

namespace delta_robot {
namespace math {

/**
 * @brief Inverse kinematics calculations for delta robot
 * Handles Steps 2 and 3: Top positions and Fermat point
 * Enhanced with Eigen integration for better performance and numerical stability
 */
class Kinematics {
public:
    /**
     * @brief Constructor with robot parameters
     */
    Kinematics(double working_height, const geometry::StaticGeometry& static_geometry);
    
    /**
     * @brief Calculate actuator top positions (Step 2)
     * @param target_point Target position [x, y, z] (already workspace-corrected)
     * @return Array of actuator heights [Az, Bz, Cz]
     */
    std::array<double, 3> calculateTopPositions(const std::array<double, 3>& target_point);
    
    /**
     * @brief Calculate Fermat point from three 3D points (Step 3)
     * @param actuator_positions Array of actuator positions [[x,y,z], [x,y,z], [x,y,z]]
     * @return Fermat point coordinates [x, y, z]
     */
    std::array<double, 3> calculateFermatPoint(
        const std::array<std::array<double, 3>, 3>& actuator_positions
    );
    
    /**
     * @brief Get intermediate calculation data for visualization/debugging
     */
    struct KinematicsData {
        std::array<double, 3> direction_vector;
        std::array<double, 3> plane_normal;
        std::array<double, 3> plane_center;
        std::array<double, 3> H_point;
        std::array<double, 3> G_point;
        std::array<double, 3> u_vector;
        
        // Fermat calculation data
        std::array<double, 3> triangle_sides;  // [a, b, c]
        std::array<double, 3> triangle_angles; // [Alpha, Beta, Gamma]
        std::array<double, 3> lambda_weights;  // [LambdaA, LambdaB, LambdaC]
    };
    
    /**
     * @brief Get intermediate data from last calculation
     */
    const KinematicsData& getLastCalculationData() const { return last_data_; }

private:
    double working_height_;
    const geometry::StaticGeometry& static_geometry_;
    KinematicsData last_data_;
    
    // Step 2 helper methods
    std::array<double, 3> createDirectionVector(const std::array<double, 3>& target_point);
    std::array<double, 3> calculatePlaneNormal(const std::array<double, 3>& direction_vector);
    std::array<double, 3> mirrorWorkingHeightPoint(
        const std::array<double, 3>& plane_center,
        const std::array<double, 3>& plane_normal
    );
    
    // Step 3 helper methods (kept for compatibility, internally use Eigen)
    std::array<double, 3> calculateTriangleSides(
        const std::array<std::array<double, 3>, 3>& points
    );
    std::array<double, 3> calculateTriangleAngles(
        const std::array<std::array<double, 3>, 3>& points,
        const std::array<double, 3>& sides
    );
    std::array<double, 3> calculateLambdaWeights(
        const std::array<double, 3>& sides,
        const std::array<double, 3>& angles
    );
};

} // namespace math
} // namespace delta_robot