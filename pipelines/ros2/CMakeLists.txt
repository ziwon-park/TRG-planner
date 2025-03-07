cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)

message(STATUS "Configuring pipelines/ros2")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/pipelines/ros2/include
)

add_executable(trg_ros2_node
    ${CMAKE_SOURCE_DIR}/pipelines/ros2/src/main.cpp
    ${CMAKE_SOURCE_DIR}/pipelines/ros2/src/ros2_node.cpp
)
ament_target_dependencies(trg_ros2_node
    rclcpp
    pcl_conversions
    pcl_ros
    nav_msgs
    visualization_msgs
)

target_link_libraries(trg_ros2_node
    planner_lib
    yaml-cpp
    ${PCL_INCLUDE_DIRS}
)
install(TARGETS
    trg_ros2_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    ${CMAKE_SOURCE_DIR}/configs
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
