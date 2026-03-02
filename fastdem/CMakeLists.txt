cmake_minimum_required(VERSION 3.14)
project(fastdem VERSION 0.3.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

###############
## Dependencies
###############

# External (System)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# External (FetchContent)
include(FetchContent)
FetchContent_Declare(nanoGrid
  GIT_REPOSITORY https://github.com/Ikhyeon-Cho/nanoGrid.git
  GIT_TAG main
)
FetchContent_MakeAvailable(nanoGrid)

# Internal (Vendored)
add_subdirectory(lib/nanoPCL)
add_subdirectory(lib/stb)

###########
## Build ##
###########

# Collect source files
file(GLOB CORE_SOURCES "src/*.cpp")

# Main library
add_library(${PROJECT_NAME}
  ${CORE_SOURCES}
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    nanoGrid::nanoGrid
    nanoPCL
    yaml-cpp
    spdlog::spdlog
  PRIVATE
    stb
)

###############
## Benchmark ##
###############

option(BUILD_BENCHMARKS "Build benchmark executables" OFF)

if(BUILD_BENCHMARKS)
  add_executable(benchmark_height_update
    benchmarks/benchmark_height_update.cpp)
  target_link_libraries(benchmark_height_update PRIVATE ${PROJECT_NAME})
  target_compile_options(benchmark_height_update PRIVATE -O3)
endif()

##############
## Examples ##
##############

option(FASTDEM_BUILD_EXAMPLES "Build fastdem examples" OFF)

if(FASTDEM_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

###########
## Tools ##
###########

option(FASTDEM_BUILD_TOOLS "Build fastdem CLI tools" OFF)

if(FASTDEM_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

###########
## Tests ##
###########

option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

#############
## Install ##
#############

include(GNUInstallDirs)

# Install library (internal targets included for linking)
install(TARGETS ${PROJECT_NAME} nanoGrid nanoPCL stb
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install headers
install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY lib/nanoPCL/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY lib/nanoPCL/thirdparty/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY lib/stb/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/stb
  FILES_MATCHING PATTERN "*.h"
)

# Install exported targets file
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Generate and install CMake config for find_package()
include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
