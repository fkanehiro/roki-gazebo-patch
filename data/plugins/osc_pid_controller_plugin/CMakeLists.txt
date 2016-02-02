cmake_minimum_required(VERSION 2.8)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(osc_pid_controller_plugin SHARED osc_pid_controller_plugin.cc)
target_link_libraries(osc_pid_controller_plugin ${GAZEBO_LIBRARIES} oscpack)

