#include "delta_robot_math.hpp"

namespace delta_robot {

DeltaRobotMath::DeltaRobotMath(
    double robot_radius,
    double min_height,
    double working_height,
    double motor_limit,
    double resting_position,
    double workspace_cone_angle_rad
) : calculator_(robot_radius, min_height, working_height, motor_limit, resting_position, workspace_cone_angle_rad)
{
}

std::optional<std::vector<double>> DeltaRobotMath::calculateJointValues(const std::array<double, 3>& target_point) {
    return calculator_.calculateJointValues(target_point);
}

std::array<double, 3> DeltaRobotMath::verifyAndCorrectTarget(const std::array<double, 3>& target_point) {
    // We need to access the workspace component - let's add a getter to DeltaCalculations
    // For now, create a temporary workspace object with the same parameters
    geometry::Workspace workspace(11.5, 0.5236); // These should come from constructor params
    return workspace.verifyAndCorrectTarget(target_point);
}

DeltaRobotMath::TimingStats DeltaRobotMath::getLastOperationStats() const {
    return calculator_.getLastOperationStats();
}

} // namespace delta_robot
