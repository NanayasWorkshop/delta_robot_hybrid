#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "delta_robot_math.hpp"
#include "delta_constants.hpp"

namespace py = pybind11;

PYBIND11_MODULE(delta_robot_cpp, m) {
    m.doc() = "Delta Robot C++ acceleration module";
    
    py::class_<delta_robot::DeltaRobotMath>(m, "DeltaRobotMath")
        .def(py::init<double, double, double, double, double, double>(),
             py::arg("robot_radius"),
             py::arg("min_height"),
             py::arg("working_height"),
             py::arg("motor_limit"),
             py::arg("resting_position"),
             py::arg("workspace_cone_angle_rad"))
        .def("calculate_joint_values", &delta_robot::DeltaRobotMath::calculateJointValues)
        .def("verify_and_correct_target", &delta_robot::DeltaRobotMath::verifyAndCorrectTarget)
        .def("get_last_operation_stats", &delta_robot::DeltaRobotMath::getLastOperationStats);
    
    py::class_<delta_robot::DeltaRobotMath::TimingStats>(m, "TimingStats")
        .def_readonly("verify_and_correct_ms", &delta_robot::DeltaRobotMath::TimingStats::verify_and_correct_ms)
        .def_readonly("calculate_top_positions_ms", &delta_robot::DeltaRobotMath::TimingStats::calculate_top_positions_ms)
        .def_readonly("calculate_fermat_ms", &delta_robot::DeltaRobotMath::TimingStats::calculate_fermat_ms)
        .def_readonly("optimization_ms", &delta_robot::DeltaRobotMath::TimingStats::optimization_ms)
        .def_readonly("total_ms", &delta_robot::DeltaRobotMath::TimingStats::total_ms);
    
    // Expose constants
    py::module constants = m.def_submodule("constants", "Delta robot constants");
    
    // Robot Physical Constants
    constants.attr("ROBOT_RADIUS") = delta_robot::constants::ROBOT_RADIUS;
    constants.attr("MIN_HEIGHT") = delta_robot::constants::MIN_HEIGHT;
    constants.attr("WORKING_HEIGHT") = delta_robot::constants::WORKING_HEIGHT;
    constants.attr("MOTOR_LIMIT") = delta_robot::constants::MOTOR_LIMIT;
    constants.attr("WORKSPACE_CONE_ANGLE_RAD") = delta_robot::constants::WORKSPACE_CONE_ANGLE_RAD;
    
    // Mathematical Constants
    constants.attr("EPSILON") = delta_robot::constants::EPSILON;
    constants.attr("MS_CONVERSION_FACTOR") = delta_robot::constants::MS_CONVERSION_FACTOR;
    
    // Geometry Constants
    constants.attr("BASE_A_ANGLE") = delta_robot::constants::BASE_A_ANGLE;
    constants.attr("BASE_B_ANGLE") = delta_robot::constants::BASE_B_ANGLE;
    constants.attr("BASE_C_ANGLE") = delta_robot::constants::BASE_C_ANGLE;
}
