cmake_minimum_required(VERSION 3.5.1)
project(grid_map_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# Find Eigen3
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

# Define Eigen addons (grid_map custom Eigen plugins)
include(cmake/${PROJECT_NAME}-extras.cmake)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/GridMap.cpp
  src/GridMapMath.cpp
  src/SubmapGeometry.cpp
  src/BufferRegion.cpp
  src/Polygon.cpp
  src/CubicInterpolation.cpp
  src/iterators/GridMapIterator.cpp
  src/iterators/SubmapIterator.cpp
  src/iterators/CircleIterator.cpp
  src/iterators/EllipseIterator.cpp
  src/iterators/SpiralIterator.cpp
  src/iterators/PolygonIterator.cpp
  src/iterators/LineIterator.cpp
  src/iterators/SlidingWindowIterator.cpp
)

# Rename output to avoid conflict with system grid_map_core
set_target_properties(${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME "grid_map_core_hm"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# Eigen3::Eigen target may not exist on older systems
if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
endif()
