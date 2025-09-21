cmake_minimum_required(VERSION 3.10)
project(pipeline_core VERSION 0.0.1 LANGUAGES CXX)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Create header-only library
add_library(${PROJECT_NAME} INTERFACE)

# Set include directories
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link yaml-cpp
target_link_libraries(${PROJECT_NAME}
  INTERFACE
    yaml-cpp
)

# Installation rules (optional)
install(DIRECTORY include/
  DESTINATION include
)

# Export target for use by other CMake projects
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)