message(STATUS "Configuring cpp")

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP) # cpp parallel programming
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Planner class library
add_library(planner_lib SHARED
    ${CMAKE_SOURCE_DIR}/cpp/src/kdtree/kdtree.c
    ${CMAKE_SOURCE_DIR}/cpp/src/planner/trg_planner.cpp
    ${CMAKE_SOURCE_DIR}/cpp/src/graph/trg.cpp
)
target_include_directories(planner_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/cpp/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(planner_lib
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)
