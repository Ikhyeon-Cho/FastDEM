cmake_minimum_required(VERSION 3.0.2)
project(height_mapping)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    grid_map_ros
    pcl_conversions
    height_map_core
    height_mapping_core
    message_generation
)

add_service_files(
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_lib
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
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Library: Height mapping algorithms (non-ROS)
add_library(${PROJECT_NAME}_lib
  src/algorithms/HeightMapper.cpp
  src/algorithms/GlobalMapper.cpp
  src/algorithms/estimators/StatMeanEstimator.cpp
  src/algorithms/estimators/KalmanEstimator.cpp
  src/algorithms/estimators/MovingAverageEstimator.cpp
  src/algorithms/filters/FastHeightFilter.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# ROS Nodes
# Node: Height mapping
add_executable(height_mapping_node
  src/ros/height_mapping_node.cpp
)

target_link_libraries(height_mapping_node
  ${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(height_mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node: Global mapping
add_executable(global_mapping_node
  src/ros/global_mapping_node.cpp
)

target_link_libraries(global_mapping_node
  ${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(global_mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Node: Sensor processor
add_executable(sensor_processor_node
  src/ros/sensor_processor_node.cpp
)

target_link_libraries(sensor_processor_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(sensor_processor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})