cmake_minimum_required(VERSION 3.14)
project(delta_robot_cpp)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Python and pybind11
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Add include directories
include_directories(include)

# Add all source files including new math modules
set(SOURCES 
    src/delta_robot_math.cpp
    src/delta_calculations.cpp
    src/geometry/static_geometry.cpp
    src/geometry/workspace.cpp
    src/geometry/coordinate_transforms.cpp
    src/utils/timing.cpp
    # Mathematical modules
    src/math/kinematics.cpp
    src/math/optimization.cpp
    src/math/orientation.cpp
    src/math/math_orchestrator.cpp
    # Eigen utilities
    src/eigen_utils.cpp
)

# Create a shared library
pybind11_add_module(delta_robot_cpp 
    src/bindings.cpp
    ${SOURCES}
)

# Link Eigen3
target_link_libraries(delta_robot_cpp PRIVATE Eigen3::Eigen)

# Set output directory
set_target_properties(delta_robot_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../python
)

# Enable compiler warnings and optimizations
if(MSVC)
    target_compile_options(delta_robot_cpp PRIVATE /W4 /O2 /Oi /fp:fast)
else()
    target_compile_options(delta_robot_cpp PRIVATE 
        -Wall -Wextra -pedantic 
        -O3 -march=native -ffast-math -finline-functions
    )
endif()

# Debug executable
add_executable(debug_test
    src/debug_main.cpp
    ${SOURCES}
)
target_include_directories(debug_test PRIVATE include)
target_link_libraries(debug_test PRIVATE Eigen3::Eigen)

# Apply same optimizations to debug executable
if(MSVC)
    target_compile_options(debug_test PRIVATE /W4 /O2 /Oi /fp:fast)
else()
    target_compile_options(debug_test PRIVATE 
        -Wall -Wextra -pedantic 
        -O3 -march=native -ffast-math -finline-functions
    )
endif()