#include "delta_robot_math.hpp"
#include "delta_constants.hpp"
#include <iostream>

int main() {
    // Calculate resting position instead of hardcoding
    double resting_position = 2 * delta_robot::constants::MOTOR_LIMIT + delta_robot::constants::MIN_HEIGHT;
    
    // Initialize the delta robot using constants
    delta_robot::DeltaRobotMath robot(
        delta_robot::constants::ROBOT_RADIUS,
        delta_robot::constants::MIN_HEIGHT,
        delta_robot::constants::WORKING_HEIGHT,
        delta_robot::constants::MOTOR_LIMIT,
        resting_position,
        delta_robot::constants::WORKSPACE_CONE_ANGLE_RAD
    );
    
    // Test cases with clear intent
    std::vector<std::array<double, 3>> test_cases = {
        {0.0, 0.0, 15.0},    // Center point
        {10.0, 0.0, 20.0},   // X-axis point
        {0.0, 10.0, 20.0},   // Y-axis point
        {10.0, 5.0, 20.0},   // Known good test case
        {15.0, 15.0, 25.0}   // Corner case
    };
    
    std::cout << "=== Delta Robot Debug Test ===" << std::endl;
    std::cout << "Robot Parameters:" << std::endl;
    std::cout << "  Radius: " << delta_robot::constants::ROBOT_RADIUS << " mm" << std::endl;
    std::cout << "  Min Height: " << delta_robot::constants::MIN_HEIGHT << " mm" << std::endl;
    std::cout << "  Working Height: " << delta_robot::constants::WORKING_HEIGHT << " mm" << std::endl;
    std::cout << "  Motor Limit: " << delta_robot::constants::MOTOR_LIMIT << " mm" << std::endl;
    std::cout << "  Resting Position: " << resting_position << " mm" << std::endl;
    std::cout << "  Workspace Cone Angle: " << delta_robot::constants::WORKSPACE_CONE_ANGLE_RAD << " rad" << std::endl;
    std::cout << std::endl;
    
    for (size_t i = 0; i < test_cases.size(); ++i) {
        const auto& test_point = test_cases[i];
        
        std::cout << "Test Case " << (i + 1) << ": [" 
                  << test_point[0] << ", " << test_point[1] << ", " << test_point[2] << "]" << std::endl;
        
        // Use the new structured result
        auto result = robot.calculateJointValues(test_point);
        
        if (result) {
            std::cout << "  Pitch: " << result->pitch << " rad" << std::endl;
            std::cout << "  Roll: " << result->roll << " rad" << std::endl;
            std::cout << "  Motor A: " << result->motor_positions[0] << " mm" << std::endl;
            std::cout << "  Motor B: " << result->motor_positions[1] << " mm" << std::endl;
            std::cout << "  Motor C: " << result->motor_positions[2] << " mm" << std::endl;
            std::cout << "  Prismatic Length: " << result->prismatic_length << " mm" << std::endl;
            std::cout << "  Fermat Point: [" << result->fermat_point[0] << ", " 
                      << result->fermat_point[1] << ", " << result->fermat_point[2] << "]" << std::endl;
            std::cout << "  Within Limits: " << (result->within_limits ? "Yes" : "No") << std::endl;
            std::cout << "  Workspace Corrected: " << (result->workspace_corrected ? "Yes" : "No") << std::endl;
            
            if (result->workspace_corrected) {
                std::cout << "  Corrected Target: [" << result->corrected_target[0] << ", " 
                          << result->corrected_target[1] << ", " << result->corrected_target[2] << "]" << std::endl;
            }
        } else {
            std::cout << "  Calculation failed!" << std::endl;
        }
        
        // Show timing stats
        auto stats = robot.getLastOperationStats();
        std::cout << "  Timing - Total: " << stats.total_ms << " ms" << std::endl;
        
        std::cout << std::endl;
    }
    
    return 0;
}