cmake_minimum_required(VERSION 3.16)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()
project(robot_self_filter_downstream_smoke LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(robot_self_filter REQUIRED)

add_executable(robot_self_filter_downstream_smoke consumer.cpp)
ament_target_dependencies(robot_self_filter_downstream_smoke robot_self_filter)
