cmake_minimum_required(VERSION 3.16...3.24)
project(trg_interface VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

include(FetchContent)

set(TRG_INTERFACE_DIR ${CMAKE_CURRENT_LIST_DIR})
message(STATUS "TRG_INTERFACE_DIR: ${TRG_INTERFACE_DIR}")
add_definitions(-DTRG_INTERFACE_DIR="${TRG_INTERFACE_DIR}")

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(trg_planner REQUIRED)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include
)

add_executable(trg_interface
    ${CMAKE_CURRENT_LIST_DIR}/src/interface.cpp
    ${CMAKE_SOURCE_DIR}/../trg_planner/core/trg_planner/src/interface/interface.cpp
)

target_include_directories(trg_interface
    PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../trg_planner/core/trg_planner/include  
)

target_link_libraries(trg_interface
    trg_planner::trg_planner_core
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
    pthread
)