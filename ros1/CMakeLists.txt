cmake_minimum_required(VERSION 3.10)
project(fastdem_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###############
## Dependencies
###############

# ROS dependencies (catkin)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2_ros
  tf2_eigen
  grid_map_msgs
  visualization_msgs
)

# Pure CMake dependency (our core library)
find_package(fastdem REQUIRED)

# OpenMP (required by fastdem)
find_package(OpenMP REQUIRED)

###############
## Catkin    ##
###############

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    tf2_ros
    grid_map_msgs
    visualization_msgs
)

###########
## Build ##
###########

# Default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Nodes   ##
#############

# Main elevation mapping node
add_executable(fastdem_node
  src/fastdem_ros_node.cpp
)

target_link_libraries(fastdem_node
  ${catkin_LIBRARIES}
  fastdem::fastdem
)

add_dependencies(fastdem_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
