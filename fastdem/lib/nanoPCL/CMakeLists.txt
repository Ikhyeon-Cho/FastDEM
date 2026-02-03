cmake_minimum_required(VERSION 3.10)
project(nanoPCL VERSION 0.1.0 LANGUAGES CXX)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =======================
# Required Dependencies
# =======================

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# =================================
# Library Definition (Header-only)
# =================================

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

# ======================
# Optional Dependencies
# ======================

# OpenMP (for parallel acceleration)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} INTERFACE OpenMP::OpenMP_CXX)
  message(STATUS "OpenMP enabled")
endif()

# Rerun (for visualization bridge)
option(USE_RERUN "Enable Rerun bridge support" OFF)
if(USE_RERUN)
  if(CMAKE_VERSION VERSION_LESS "3.14")
    message(FATAL_ERROR "Rerun bridge requires CMake >= 3.14 (for FetchContent)")
  endif()

  find_package(rerun_sdk CONFIG QUIET)
  if(NOT rerun_sdk_FOUND)
    message(STATUS "Rerun SDK not found. Fetching via FetchContent...")
    include(FetchContent)
    set(RERUN_VERSION "0.28.2")
    FetchContent_Declare(
      rerun_sdk
      URL https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip
    )
    FetchContent_MakeAvailable(rerun_sdk)
  endif()

  if(TARGET rerun_sdk)
    target_link_libraries(${PROJECT_NAME} INTERFACE rerun_sdk)
    target_compile_definitions(${PROJECT_NAME} INTERFACE NANOPCL_USE_RERUN)
    message(STATUS "Rerun bridge enabled")
    message(STATUS "  Note: Rerun Viewer required (pip install rerun-sdk)")
  endif()
endif()

# =======================================
# Examples, Benchmarks, Tests (Optional)
# =======================================

option(BUILD_EXAMPLES "Build examples" OFF)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

option(BUILD_BENCHMARKS "Build benchmarks (requires PCL)" OFF)
if(BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# =============
# Installation
# =============

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

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
