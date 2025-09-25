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

# Automatically collect all source files
file(GLOB ALGORITHM_SOURCES "src/algorithms/*.cpp")
file(GLOB ESTIMATOR_SOURCES "src/estimators/*.cpp")

# Core library with algorithms and estimators
add_library(${PROJECT_NAME}
  ${ALGORITHM_SOURCES}  # Core algorithms
  ${ESTIMATOR_SOURCES}  # Estimator implementations
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
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