#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "delta_robot_math.hpp"

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
}
