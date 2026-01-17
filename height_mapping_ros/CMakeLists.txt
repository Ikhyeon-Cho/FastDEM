cmake_minimum_required(VERSION 3.10)
project(height_mapping_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###############
## Dependencies
###############

# ROS dependencies (catkin)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_eigen
  grid_map_msgs
  message_generation
  rosbag
)

# Pure CMake dependency (our core library)
find_package(height_mapping REQUIRED)

# OpenMP (required by height_mapping)
find_package(OpenMP REQUIRED)
message(STATUS "height_mapping_INCLUDE_DIRS: ${height_mapping_INCLUDE_DIRS}")
message(STATUS "height_mapping_LIBRARIES: ${height_mapping_LIBRARIES}")
message(STATUS "height_mapping_LIBRARY_DIRS: ${height_mapping_LIBRARY_DIRS}")

###############
## Messages  ##
###############

add_service_files(
  FILES
  save_map.srv
)
generate_messages()

###############
## Catkin    ##
###############

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    tf2_ros
    grid_map_msgs
    message_runtime
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

# Main height mapping node
add_executable(height_mapping_node
  src/height_mapping_node.cpp
)

target_link_libraries(height_mapping_node
  ${catkin_LIBRARIES}
  height_mapping::height_mapping
)

add_dependencies(height_mapping_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Experimental: PPL mapping node (pipeline-based)
add_executable(ppl_mapping_node
  experimental/src/ppl_mapping_node.cpp
)

target_link_libraries(ppl_mapping_node
  ${catkin_LIBRARIES}
  height_mapping::height_mapping
)

add_dependencies(ppl_mapping_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

