cmake_minimum_required(VERSION 3.0.2)
project(height_mapping_pipeline)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  height_mapping_core
)

# Add flowpipelib as subdirectory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../flowpipelib ${CMAKE_CURRENT_BINARY_DIR}/flowpipelib)

catkin_package(
  INCLUDE_DIRS
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../flowpipelib/include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    height_mapping_core
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
  ${catkin_INCLUDE_DIRS}
)

# Pipeline library
add_library(${PROJECT_NAME}
  src/mapping_engine.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  flowpipelib
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)