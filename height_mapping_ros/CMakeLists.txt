cmake_minimum_required(VERSION 3.0.2)
project(height_mapping)

add_compile_options(-std=c++17)

###################
## Find packages ##
###################
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    tf2_eigen
    grid_map_ros
    pcl_conversions
    height_map_core
    height_mapping_core
    message_generation
)

####################
## Service files  ##
####################
add_service_files(
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    tf2_ros
    grid_map_ros
    pcl_conversions
    height_map_core
    height_mapping_core
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
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

################
## Libraries  ##
################
add_library(height_mapping_utils
  src/utils/pointcloud_converter.cpp
)
target_link_libraries(height_mapping_utils
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Nodes   ##
#############
# Main height mapping node
add_executable(height_mapping_node
  src/height_mapping_node.cpp
)
target_link_libraries(height_mapping_node
  height_mapping_utils
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(height_mapping_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)