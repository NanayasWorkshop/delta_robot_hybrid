cmake_minimum_required(VERSION 3.14)
project(delta_robot_cpp)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Python and pybind11
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Add include directories
include_directories(include)

# Add all source files
set(SOURCES 
    src/delta_robot_math.cpp
    src/delta_calculations.cpp
    src/geometry/static_geometry.cpp
    src/geometry/workspace.cpp
    src/geometry/coordinate_transforms.cpp
    src/utils/timing.cpp
)

# Create a shared library
pybind11_add_module(delta_robot_cpp 
    src/bindings.cpp
    ${SOURCES}
)

# Set output directory
set_target_properties(delta_robot_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../python
)

# Enable compiler warnings
if(MSVC)
    target_compile_options(delta_robot_cpp PRIVATE /W4)
else()
    target_compile_options(delta_robot_cpp PRIVATE -Wall -Wextra -pedantic)
endif()
