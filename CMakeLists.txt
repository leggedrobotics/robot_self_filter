cmake_minimum_required(VERSION 3.8)
project(robot_self_filter)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#
# Find dependencies
#
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

# ROS 2 packages
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs QUIET)  # optional
find_package(pcl_ros REQUIRED) # or use pure PCL, depending on your usage
find_package(filters REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML REQUIRED)
# Bullet
find_package(Bullet REQUIRED)

# Assimp
find_package(assimp QUIET)
if(NOT assimp_FOUND)
  message(WARNING "Assimp not found by find_package. Trying pkg-config fallback.")
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(ASSIMP REQUIRED assimp)
endif()

#
# Include dirs
#
# Depending on how your system or FindBullet.cmake is set up, these might be
# BULLET_INCLUDE_DIR (singular) or bullet_INCLUDE_DIRS (lowercase).
# Check which variables are actually populated in your environment.
include_directories(
  include
  ${BULLET_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${FILTERS_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

#
# Libraries & definitions
#
add_definitions(${PCL_DEFINITIONS})
if(ASSIMP_FOUND)
  # If the version is above 2.0, set -DASSIMP_UNIFIED_HEADER_NAMES
  if(NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150")
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
    message(STATUS "Assimp version has unified headers")
  else()
    message(STATUS "Assimp version does not have unified headers")
  endif()
endif()

#
# Create libraries
#
add_library(robot_geometric_shapes
  src/load_mesh.cpp
  src/shapes.cpp
  src/bodies.cpp
)

target_link_libraries(robot_geometric_shapes
  ${BULLET_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/self_mask.cpp
)

target_link_libraries(${PROJECT_NAME}
  robot_geometric_shapes
  ${ASSIMP_LIBRARIES}
)

#
# Create executables
#
# add_executable(test_filter
#   src/test_filter.cpp
# )

# target_link_libraries(test_filter
#   ${PROJECT_NAME}
#   robot_geometric_shapes
#   ${ASSIMP_LIBRARIES}
# )

add_executable(self_filter
  src/self_filter.cpp
)

target_link_libraries(self_filter
  ${PROJECT_NAME}
  robot_geometric_shapes
  ${ASSIMP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

#
# Link dependencies
#
ament_target_dependencies(robot_geometric_shapes
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  urdf
  resource_retriever
  visualization_msgs
  pcl_ros  # or pcl_conversions
  filters
  tinyxml_vendor
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  urdf
  resource_retriever
  visualization_msgs
  pcl_ros  # or pcl_conversions
  filters
  tinyxml_vendor
)

# ament_target_dependencies(test_filter
#   rclcpp
#   tf2
#   tf2_ros
#   geometry_msgs
#   sensor_msgs
#   urdf
#   resource_retriever
#   visualization_msgs
#   pcl_ros  # or pcl_conversions
#   filters
# )

ament_target_dependencies(self_filter
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  urdf
  resource_retriever
  visualization_msgs
  pcl_ros  # or pcl_conversions
  filters
  tinyxml_vendor
)

#
# Install
#
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    params
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    robot_geometric_shapes
    ${PROJECT_NAME}
#    test_filter
    self_filter
  DESTINATION lib/${PROJECT_NAME}
)

#
# Export and finalize
#
ament_export_include_directories(
  include
)

ament_export_libraries(
  robot_geometric_shapes
  ${PROJECT_NAME}
)

# Exporting Bullet as a standard ament dependency doesn't typically do anything,
# because Bullet is not a ROS package. It's fine to omit it.
ament_export_dependencies(
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  urdf
  resource_retriever
  visualization_msgs
  pcl_ros
  assimp
  tinyxml_vendor
  # bullet  <-- Usually remove this, unless you have a custom bullet ROS package
)

ament_package()
