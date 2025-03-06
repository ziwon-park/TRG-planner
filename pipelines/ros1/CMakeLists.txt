cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "Configuring pipelines/ros1")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
    grid_map_ros
)

# set (PCL_DIR "/usr/include/pcl-1.10")
find_package(PCL 1.10 REQUIRED)
# set (PCL_DIR "/usr/local/include/pcl-1.14")
# find_package(PCL 1.14 REQUIRED)
message(STATUS "PCL version: ${PCL_VERSION}")
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")
find_package(OpenCV REQUIRED)
message(STATUS "OPENCV version: ${OpenCV_VERSION}")

catkin_package()

include_directories(
    ${CMAKE_SOURCE_DIR}/pipelines/ros1/include
    ${catkin_INCLUDE_DIRS}
)

add_executable(trg_ros1_node 
    ${CMAKE_SOURCE_DIR}/pipelines/ros1/src/main.cpp
    ${CMAKE_SOURCE_DIR}/pipelines/ros1/src/ros1_node.cpp
)
target_link_libraries(trg_ros1_node
    ${catkin_LIBRARIES}
    planner_lib
)