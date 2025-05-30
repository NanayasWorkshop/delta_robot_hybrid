#include "delta_calculations.hpp"

namespace delta_robot {

DeltaCalculations::DeltaCalculations(
    double robot_radius,
    double min_height,
    double working_height,
    double motor_limit,
    double resting_position,
    double workspace_cone_angle_rad
) : orchestrator_(robot_radius, min_height, working_height, motor_limit, resting_position, workspace_cone_angle_rad)
{
}

std::optional<CalculationResult> DeltaCalculations::calculateJointValues(const std::array<double, 3>& target_point) {
    return orchestrator_.calculateJointValues(target_point);
}

std::optional<std::vector<double>> DeltaCalculations::calculateJointValuesLegacy(const std::array<double, 3>& target_point) {
    auto result = orchestrator_.calculateJointValues(target_point);
    if (!result) {
        return std::nullopt;
    }
    
    return resultToLegacyVector(*result);
}

utils::TimingStats DeltaCalculations::getLastOperationStats() const {
    return orchestrator_.getLastOperationStats();
}

std::vector<double> DeltaCalculations::resultToLegacyVector(const CalculationResult& result) const {
    return {
        result.pitch,                           // [0]
        result.roll,                            // [1]
        result.motor_positions[0],              // [2]
        result.motor_positions[1],              // [3]
        result.motor_positions[2],              // [4]
        result.prismatic_length,                // [5]
        result.fermat_point[0],                 // [6]
        result.fermat_point[1],                 // [7]
        result.fermat_point[2],                 // [8]
        static_cast<double>(result.within_limits),        // [9]
        result.original_target[0],              // [10]
        result.original_target[1],              // [11]
        result.original_target[2],              // [12]
        result.corrected_target[0],             // [13]
        result.corrected_target[1],             // [14]
        result.corrected_target[2],             // [15]
        static_cast<double>(result.workspace_corrected)   // [16]
    };
}

} // namespace delta_robot