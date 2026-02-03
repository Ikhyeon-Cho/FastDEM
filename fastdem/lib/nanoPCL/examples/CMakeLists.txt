cmake_minimum_required(VERSION 3.14)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# =============================================================================
# Core Examples
# =============================================================================

set(EXAMPLE_SOURCES
  01_quick_start
  02_channels
  03_filtering
  04_registration
  05_segmentation
  06_io
)

foreach(example ${EXAMPLE_SOURCES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE nanoPCL::nanoPCL)
  target_compile_definitions(${example} PRIVATE
    NANOPCL_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
    NANOPCL_EXAMPLES_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
endforeach()

# =============================================================================
# Rerun Examples (requires USE_RERUN=ON)
# =============================================================================

add_subdirectory(rerun)
