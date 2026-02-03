# nanoPCL Benchmarks
# These benchmarks are optional and NOT part of the core library.
#
# Usage:
#   mkdir build && cd build
#   cmake ..                              # nanopcl benchmarks only
#   cmake .. -DBUILD_VS_PCL=ON            # + PCL comparison (requires PCL)
#   cmake .. -DBUILD_VS_SMALL_GICP=ON     # + small_gicp comparison

cmake_minimum_required(VERSION 3.14)
project(nanoPCL_Benchmarks)

# C++17 required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Options
# =============================================================================
option(BUILD_VS_PCL "Build PCL comparison benchmarks (requires PCL)" OFF)
option(BUILD_VS_SMALL_GICP "Build small_gicp comparison benchmarks" OFF)

# =============================================================================
# Common Dependencies
# =============================================================================
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP QUIET)

# nanoPCL - use local source (header-only)
set(NANOPCL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
set(NANOPCL_THIRDPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty)

if(NOT EXISTS ${NANOPCL_INCLUDE_DIR}/nanopcl/common.hpp)
  message(FATAL_ERROR "nanoPCL headers not found at ${NANOPCL_INCLUDE_DIR}")
endif()

# Create interface library for benchmarks
add_library(nanoPCL_bench INTERFACE)
target_include_directories(nanoPCL_bench INTERFACE
  ${NANOPCL_INCLUDE_DIR}
  ${NANOPCL_THIRDPARTY_DIR}
)
target_link_libraries(nanoPCL_bench INTERFACE Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(nanoPCL_bench INTERFACE OpenMP::OpenMP_CXX)
endif()

# Common include path
set(BENCH_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/common)

# Helper function to create benchmark targets
function(add_benchmark NAME SOURCE)
  add_executable(${NAME} ${SOURCE})
  target_link_libraries(${NAME} PRIVATE nanoPCL_bench)
  target_include_directories(${NAME} PRIVATE ${BENCH_COMMON_DIR})
  target_compile_options(${NAME} PRIVATE -O3 -march=native)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(${NAME} PRIVATE OpenMP::OpenMP_CXX)
  endif()
endfunction()

# =============================================================================
# nanopcl/ - Core benchmarks (no external dependencies)
# =============================================================================
message(STATUS "Building nanoPCL internal benchmarks")

# Working benchmarks
add_benchmark(bench_voxel_grid     nanopcl/benchmark_voxel_grid.cpp)
add_benchmark(bench_crop_angle     nanopcl/benchmark_crop_angle.cpp)
add_benchmark(bench_pcd_io         nanopcl/benchmark_pcd_io.cpp)
# add_benchmark(bench_ransac_plane   nanopcl/benchmark_ransac_plane.cpp)  # Eigen size mismatch in segmentation
add_benchmark(bench_plane_distance nanopcl/benchmark_plane_distance.cpp)

# Re-enabling after API update
add_benchmark(bench_filter         nanopcl/benchmark_filter.cpp)
add_benchmark(bench_subset         nanopcl/benchmark_subset_extraction.cpp)
add_benchmark(bench_transform      nanopcl/benchmark_transform.cpp)
# ICP benchmarks
add_benchmark(bench_icp_p2p_vs_p2plane nanopcl/benchmark_icp_p2p_vs_p2plane.cpp)
add_benchmark(bench_icp_all        nanopcl/benchmark_icp_all.cpp)
add_benchmark(bench_icp_kitti      nanopcl/benchmark_icp_kitti.cpp)

# VoxelGrid KITTI
add_benchmark(bench_voxel_grid_kitti nanopcl/benchmark_voxel_grid_kitti.cpp)

if(PCL_FOUND)
  target_link_libraries(bench_icp_kitti PRIVATE ${PCL_LIBRARIES})
  target_include_directories(bench_icp_kitti PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(bench_icp_kitti PRIVATE HAVE_PCL ${PCL_DEFINITIONS})

  target_link_libraries(bench_voxel_grid_kitti PRIVATE ${PCL_LIBRARIES})
  target_include_directories(bench_voxel_grid_kitti PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(bench_voxel_grid_kitti PRIVATE HAVE_PCL ${PCL_DEFINITIONS})
endif()

# =============================================================================
# pcl/ - PCL comparison benchmarks (optional)
# =============================================================================
if(BUILD_VS_PCL)
  find_package(PCL QUIET COMPONENTS common features search filters registration)

  if(PCL_FOUND)
    message(STATUS "PCL ${PCL_VERSION} found - building comparison benchmarks")

    function(add_pcl_benchmark NAME SOURCE)
      add_executable(${NAME} ${SOURCE})
      target_link_libraries(${NAME} PRIVATE nanoPCL_bench ${PCL_LIBRARIES})
      target_include_directories(${NAME} PRIVATE ${BENCH_COMMON_DIR} ${PCL_INCLUDE_DIRS})
      target_compile_definitions(${NAME} PRIVATE ${PCL_DEFINITIONS})
      target_compile_options(${NAME} PRIVATE -O3 -march=native)
      if(OpenMP_CXX_FOUND)
        target_link_libraries(${NAME} PRIVATE OpenMP::OpenMP_CXX)
      endif()
    endfunction()

    add_pcl_benchmark(bench_pcl_filter     pcl/benchmark_filter.cpp)
    add_pcl_benchmark(bench_pcl_pointcloud pcl/benchmark_pointcloud.cpp)
    add_pcl_benchmark(bench_pcl_normal     pcl/benchmark_normal.cpp)
    add_pcl_benchmark(bench_pcl_sor        pcl/benchmark_sor.cpp)
    add_pcl_benchmark(bench_pcl_icp        pcl/benchmark_icp.cpp)
    add_pcl_benchmark(bench_pcl_ransac     pcl/benchmark_ransac_plane.cpp)
    add_pcl_benchmark(bench_pcl_ransac_accuracy pcl/benchmark_ransac_accuracy.cpp)
    add_pcl_benchmark(bench_pcl_voxel_accuracy  pcl/benchmark_voxel_accuracy.cpp)
  else()
    message(WARNING "BUILD_VS_PCL=ON but PCL not found. Skipping PCL benchmarks.")
  endif()
else()
  message(STATUS "PCL comparison benchmarks disabled (use -DBUILD_VS_PCL=ON)")
endif()

# =============================================================================
# small_gicp/ - small_gicp comparison benchmarks (optional)
# =============================================================================
if(BUILD_VS_SMALL_GICP)
  # small_gicp is header-only, user provides path via SMALL_GICP_DIR
  if(NOT DEFINED SMALL_GICP_DIR)
    # Try common locations
    set(_small_gicp_search_paths
      ${CMAKE_CURRENT_SOURCE_DIR}/../../small_gicp
      ${CMAKE_SOURCE_DIR}/../small_gicp
      $ENV{HOME}/small_gicp
    )
    foreach(_path ${_small_gicp_search_paths})
      if(EXISTS ${_path}/include/small_gicp)
        set(SMALL_GICP_DIR ${_path})
        break()
      endif()
    endforeach()
  endif()

  if(SMALL_GICP_DIR AND EXISTS ${SMALL_GICP_DIR}/include/small_gicp)
    message(STATUS "small_gicp found at ${SMALL_GICP_DIR} - building comparison benchmarks")

    function(add_small_gicp_benchmark NAME SOURCE)
      add_executable(${NAME} ${SOURCE})
      target_link_libraries(${NAME} PRIVATE nanoPCL_bench)
      target_include_directories(${NAME} PRIVATE
        ${BENCH_COMMON_DIR}
        ${SMALL_GICP_DIR}/include
      )
      target_compile_options(${NAME} PRIVATE -O3 -march=native)
      if(OpenMP_CXX_FOUND)
        target_link_libraries(${NAME} PRIVATE OpenMP::OpenMP_CXX)
      endif()
    endfunction()

    add_small_gicp_benchmark(bench_small_gicp_voxelgrid small_gicp/bench_voxelgrid.cpp)
    add_small_gicp_benchmark(bench_small_gicp_normal    small_gicp/bench_normal_estimation.cpp)
  else()
    message(WARNING "BUILD_VS_SMALL_GICP=ON but small_gicp not found.")
    message(WARNING "Set -DSMALL_GICP_DIR=/path/to/small_gicp")
  endif()
else()
  message(STATUS "small_gicp comparison benchmarks disabled (use -DBUILD_VS_SMALL_GICP=ON)")
endif()

# =============================================================================
# Summary
# =============================================================================
message(STATUS "")
message(STATUS "nanoPCL Benchmarks Configuration:")
message(STATUS "  - Internal benchmarks: ON")
message(STATUS "  - PCL comparison:      ${BUILD_VS_PCL}")
message(STATUS "  - small_gicp comparison: ${BUILD_VS_SMALL_GICP}")
message(STATUS "")
