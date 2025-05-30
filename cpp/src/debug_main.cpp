#include "delta_robot_math.hpp"
#include <iostream>

int main() {
    // Initialize the delta robot
    delta_robot::DeltaRobotMath robot(
        24.8,  // robot_radius
        101.0, // min_height
        11.5,  // working_height
        11.0,  // motor_limit
        123.0, // resting_position
        0.5236 // workspace_cone_angle_rad
    );
    
    // Test a simple point
    std::array<double, 3> test_point = {10.0, 5.0, 20.0};
    
    std::cout << "Testing point: [" << test_point[0] << ", " 
              << test_point[1] << ", " << test_point[2] << "]" << std::endl;
    
    // This is where the math happens - you can set breakpoints here!
    auto result = robot.calculateJointValues(test_point);
    
    if (result) {
        std::cout << "Pitch: " << (*result)[0] << std::endl;
        std::cout << "Roll: " << (*result)[1] << std::endl;
        std::cout << "Motor A: " << (*result)[2] << std::endl;
        std::cout << "Motor B: " << (*result)[3] << std::endl;
        std::cout << "Motor C: " << (*result)[4] << std::endl;
    } else {
        std::cout << "Calculation failed!" << std::endl;
    }
    
    return 0;
}