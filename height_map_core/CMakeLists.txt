cmake_minimum_required(VERSION 3.0.2)
project(height_map_core)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    grid_map_core
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    grid_map_core
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

link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/HeightMap.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)