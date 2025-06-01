#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "delta_robot_math.hpp"
#include "delta_constants.hpp"
#include "math/math_orchestrator.hpp"
#include "math/kinematics.hpp"

namespace py = pybind11;

PYBIND11_MODULE(delta_robot_cpp, m) {
    m.doc() = "Delta Robot C++ acceleration module";
    
    // Bind the CalculationResult structure
    py::class_<delta_robot::CalculationResult>(m, "CalculationResult")
        .def(py::init<>())
        .def_readwrite("pitch", &delta_robot::CalculationResult::pitch)
        .def_readwrite("roll", &delta_robot::CalculationResult::roll)
        .def_readwrite("motor_positions", &delta_robot::CalculationResult::motor_positions)
        .def_readwrite("prismatic_length", &delta_robot::CalculationResult::prismatic_length)
        .def_readwrite("fermat_point", &delta_robot::CalculationResult::fermat_point)
        .def_readwrite("within_limits", &delta_robot::CalculationResult::within_limits)
        .def_readwrite("original_target", &delta_robot::CalculationResult::original_target)
        .def_readwrite("corrected_target", &delta_robot::CalculationResult::corrected_target)
        .def_readwrite("workspace_corrected", &delta_robot::CalculationResult::workspace_corrected);
    
    // NEW: Bind KinematicsData structure for math visualization
    py::class_<delta_robot::math::Kinematics::KinematicsData>(m, "KinematicsData")
        .def(py::init<>())
        .def_readwrite("direction_vector", &delta_robot::math::Kinematics::KinematicsData::direction_vector)
        .def_readwrite("plane_normal", &delta_robot::math::Kinematics::KinematicsData::plane_normal)
        .def_readwrite("plane_center", &delta_robot::math::Kinematics::KinematicsData::plane_center)
        .def_readwrite("H_point", &delta_robot::math::Kinematics::KinematicsData::H_point)
        .def_readwrite("G_point", &delta_robot::math::Kinematics::KinematicsData::G_point)
        .def_readwrite("u_vector", &delta_robot::math::Kinematics::KinematicsData::u_vector)
        .def_readwrite("triangle_sides", &delta_robot::math::Kinematics::KinematicsData::triangle_sides)
        .def_readwrite("triangle_angles", &delta_robot::math::Kinematics::KinematicsData::triangle_angles)
        .def_readwrite("lambda_weights", &delta_robot::math::Kinematics::KinematicsData::lambda_weights);
    
    // NEW: Bind Kinematics class (minimal - just what we need for visualization)
    py::class_<delta_robot::math::Kinematics>(m, "Kinematics")
        .def("get_last_calculation_data", &delta_robot::math::Kinematics::getLastCalculationData,
             py::return_value_policy::reference_internal);
    
    // Bind the main DeltaRobotMath class
    py::class_<delta_robot::DeltaRobotMath>(m, "DeltaRobotMath")
        .def(py::init<double, double, double, double, double, double>(),
             py::arg("robot_radius"),
             py::arg("min_height"),
             py::arg("working_height"),
             py::arg("motor_limit"),
             py::arg("resting_position"),
             py::arg("workspace_cone_angle_rad"))
        .def("calculate_joint_values", &delta_robot::DeltaRobotMath::calculateJointValues,
             "Calculate joint values returning CalculationResult structure")
        .def("calculate_joint_values_legacy", &delta_robot::DeltaRobotMath::calculateJointValuesLegacy,
             "Calculate joint values returning legacy vector format (deprecated)")
        .def("verify_and_correct_target", &delta_robot::DeltaRobotMath::verifyAndCorrectTarget)
        .def("get_last_operation_stats", &delta_robot::DeltaRobotMath::getLastOperationStats)
        .def("get_kinematics", [](const delta_robot::DeltaRobotMath& self) -> const delta_robot::math::Kinematics& {
            return self.getMathOrchestrator().getKinematics();
        }, py::return_value_policy::reference_internal, "Get access to kinematics module for visualization");
    
    // Bind TimingStats
    py::class_<delta_robot::DeltaRobotMath::TimingStats>(m, "TimingStats")
        .def_readonly("verify_and_correct_ms", &delta_robot::DeltaRobotMath::TimingStats::verify_and_correct_ms)
        .def_readonly("calculate_top_positions_ms", &delta_robot::DeltaRobotMath::TimingStats::calculate_top_positions_ms)
        .def_readonly("calculate_fermat_ms", &delta_robot::DeltaRobotMath::TimingStats::calculate_fermat_ms)
        .def_readonly("optimization_ms", &delta_robot::DeltaRobotMath::TimingStats::optimization_ms)
        .def_readonly("total_ms", &delta_robot::DeltaRobotMath::TimingStats::total_ms);
    
    // Expose constants
    py::module_ constants = m.def_submodule("constants", "Delta robot constants");
    
    // Robot Physical Constants
    constants.attr("ROBOT_RADIUS") = delta_robot::constants::ROBOT_RADIUS;
    constants.attr("MIN_HEIGHT") = delta_robot::constants::MIN_HEIGHT;
    constants.attr("WORKING_HEIGHT") = delta_robot::constants::WORKING_HEIGHT;
    constants.attr("MOTOR_LIMIT") = delta_robot::constants::MOTOR_LIMIT;
    constants.attr("WORKSPACE_CONE_ANGLE_RAD") = delta_robot::constants::WORKSPACE_CONE_ANGLE_RAD;
    
    // Mathematical Constants
    constants.attr("EPSILON") = delta_robot::constants::EPSILON;
    constants.attr("MS_CONVERSION_FACTOR") = delta_robot::constants::MS_CONVERSION_FACTOR;
    
    // Tolerance Constants
    constants.attr("COORDINATE_TOLERANCE") = delta_robot::constants::COORDINATE_TOLERANCE;
    constants.attr("GEOMETRIC_TOLERANCE") = delta_robot::constants::GEOMETRIC_TOLERANCE;
    constants.attr("VECTOR_MAGNITUDE_TOLERANCE") = delta_robot::constants::VECTOR_MAGNITUDE_TOLERANCE;
    constants.attr("ANGLE_TOLERANCE") = delta_robot::constants::ANGLE_TOLERANCE;
    constants.attr("MIN_DISTANCE_TOLERANCE") = delta_robot::constants::MIN_DISTANCE_TOLERANCE;
    constants.attr("WORKSPACE_DISTANCE_TOLERANCE") = delta_robot::constants::WORKSPACE_DISTANCE_TOLERANCE;
    
    // Trigonometric Constants
    constants.attr("TRIG_CLAMP_MIN") = delta_robot::constants::TRIG_CLAMP_MIN;
    constants.attr("TRIG_CLAMP_MAX") = delta_robot::constants::TRIG_CLAMP_MAX;
    
    // Fermat Point Constants
    constants.attr("FERMAT_ANGLE_OFFSET") = delta_robot::constants::FERMAT_ANGLE_OFFSET;
    constants.attr("FERMAT_MIN_DENOMINATOR") = delta_robot::constants::FERMAT_MIN_DENOMINATOR;
    
    // Geometry Constants
    constants.attr("BASE_A_ANGLE") = delta_robot::constants::BASE_A_ANGLE;
    constants.attr("BASE_B_ANGLE") = delta_robot::constants::BASE_B_ANGLE;
    constants.attr("BASE_C_ANGLE") = delta_robot::constants::BASE_C_ANGLE;
}