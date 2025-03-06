cmake_minimum_required(VERSION 3.10)
project(trg_planner VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

# Check build types
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
            STRING "Choose the type of build." FORCE)
endif ()

# set package directory path
set(TRG_DIR ${CMAKE_SOURCE_DIR})
add_definitions(-DTRG_DIR="${TRG_DIR}")

# ROS 1
if (DEFINED CATKIN_DEVEL_PREFIX)
    message(STATUS "ROS1 build")

    find_package(catkin REQUIRED COMPONENTS roscpp)

    add_subdirectory(cpp)
    add_subdirectory(pipelines/ros1)

# ROS 2
elseif (DEFINED ENV{AMENT_PREFIX_PATH})
    message(STATUS "ROS2 build")

    find_package(rclcpp REQUIRED)

    add_subdirectory(cpp)
    add_subdirectory(pipelines/ros2)

# Cmake
else()
    message(STATUS "C++ build")

    add_subdirectory(cpp)
    add_subdirectory(pipelines/cpp)

endif()
