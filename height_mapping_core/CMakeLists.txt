cmake_minimum_required(VERSION 3.0.2)
project(height_mapping_core)
add_compile_options(-std=c++17)

find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    grid_map_core
)

# Add pipeline_core as subdirectory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../pipeline_core ${CMAKE_CURRENT_BINARY_DIR}/pipeline_core)

catkin_package(
  INCLUDE_DIRS
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../pipeline_core/include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    grid_map_core
)

###########
## Build ##
###########
# Allow choosing build type via environment variable
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
endif()

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../pipeline_core/include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})

# Core library with engine implementation
add_library(${PROJECT_NAME}
  src/mapping_engine.cpp
  # Estimator implementations
  src/estimators/estimator_factory.cpp
  src/estimators/incremental_mean.cpp
  src/estimators/kalman_filter.cpp
  src/estimators/moving_average.cpp
  # Stage implementations
  src/stages/voxel_filter.cpp
  src/stages/transform_cloud.cpp
  src/stages/passthrough_filter.cpp
  src/stages/raycasting.cpp
  src/stages/height_estimation.cpp
  src/stages/multi_sensor_sync.cpp
  src/stages/move_origin.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  pipeline_core
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..
)

# Testing section - tests will be added here