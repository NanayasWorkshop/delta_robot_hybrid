#include "geometry/static_geometry.hpp"
#include "delta_constants.hpp"
#include <cmath>

namespace delta_robot {
namespace geometry {

StaticGeometry::StaticGeometry(double robot_radius) 
    : robot_radius_(robot_radius) {
    calculateBasePositions();
}

const std::array<std::array<double, 2>, 3>& StaticGeometry::getBasePositions() const {
    return base_positions_;
}

const std::array<double, 2>& StaticGeometry::getBasePosition(int index) const {
    return base_positions_[index];
}

double StaticGeometry::getRobotRadius() const {
    return robot_radius_;
}

void StaticGeometry::calculateBasePositions() {
    // Use constants instead of hardcoded values
    
    // Base A at 0 degrees (top)
    double Ar_x = robot_radius_ * std::cos(constants::BASE_A_ANGLE);
    double Ar_y = robot_radius_ * std::sin(constants::BASE_A_ANGLE);
    
    // Base B at -30 degrees (bottom right)
    double Br_x = robot_radius_ * std::cos(constants::BASE_B_ANGLE);
    double Br_y = robot_radius_ * std::sin(constants::BASE_B_ANGLE);
    
    // Base C at -150 degrees (bottom left)  
    double Cr_x = robot_radius_ * std::cos(constants::BASE_C_ANGLE);
    double Cr_y = robot_radius_ * std::sin(constants::BASE_C_ANGLE);
    
    base_positions_ = {
        std::array<double, 2>{Ar_x, Ar_y},
        std::array<double, 2>{Br_x, Br_y},
        std::array<double, 2>{Cr_x, Cr_y}
    };
}

} // namespace geometry
} // namespace delta_robot
