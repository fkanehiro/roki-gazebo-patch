cmake_minimum_required(VERSION 2.8)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(pid_position_player_plugin SHARED pid_position_player_plugin.cc)
target_link_libraries(pid_position_player_plugin ${GAZEBO_LIBRARIES})

