cmake_minimum_required(VERSION 3.16)
project(fastdem_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
endif()

###############
## Dependencies
###############

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Pure CMake dependency (our core library)
find_package(fastdem REQUIRED)

# yaml-cpp (for config loading)
find_package(yaml-cpp REQUIRED)

# OpenMP (required by fastdem)
find_package(OpenMP REQUIRED)

###########
## Build ##
###########

# Main elevation mapping node
add_executable(fastdem_node
  src/fastdem_ros_node.cpp
)

target_include_directories(fastdem_node PRIVATE
  include
)

ament_target_dependencies(fastdem_node
  rclcpp
  tf2_ros
  tf2_eigen
  sensor_msgs
  std_srvs
  grid_map_msgs
  visualization_msgs
)

target_link_libraries(fastdem_node
  fastdem::fastdem
  yaml-cpp
  OpenMP::OpenMP_CXX
)

#############
## Install ##
#############

install(TARGETS fastdem_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
