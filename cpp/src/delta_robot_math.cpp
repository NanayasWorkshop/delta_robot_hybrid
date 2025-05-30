#include "delta_robot_math.hpp"
#include "delta_constants.hpp"

namespace delta_robot {

DeltaRobotMath::DeltaRobotMath(
    double robot_radius,
    double min_height,
    double working_height,
    double motor_limit,
    double resting_position,
    double workspace_cone_angle_rad
) : calculator_(robot_radius, min_height, working_height, motor_limit, resting_position, workspace_cone_angle_rad),
    working_height_(working_height),
    workspace_cone_angle_rad_(workspace_cone_angle_rad)
{
}

std::optional<CalculationResult> DeltaRobotMath::calculateJointValues(const std::array<double, 3>& target_point) {
    return calculator_.calculateJointValues(target_point);
}

std::optional<std::vector<double>> DeltaRobotMath::calculateJointValuesLegacy(const std::array<double, 3>& target_point) {
    return calculator_.calculateJointValuesLegacy(target_point);
}

std::array<double, 3> DeltaRobotMath::verifyAndCorrectTarget(const std::array<double, 3>& target_point) {
    // Use the correct parameters from constructor instead of hardcoded values
    geometry::Workspace workspace(working_height_, workspace_cone_angle_rad_);
    return workspace.verifyAndCorrectTarget(target_point);
}

DeltaRobotMath::TimingStats DeltaRobotMath::getLastOperationStats() const {
    return calculator_.getLastOperationStats();
}

} // namespace delta_robot