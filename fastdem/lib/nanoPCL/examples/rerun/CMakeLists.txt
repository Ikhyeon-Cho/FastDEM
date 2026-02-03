# nanoPCL Rerun Examples
# Visualization versions of core examples (requires USE_RERUN=ON)

if(USE_RERUN)
  message(STATUS "Building Rerun examples")

  set(RERUN_EXAMPLES
    01_quick_start
    02_channels
    03_filtering
    04_registration
    05_segmentation
  )

  foreach(example ${RERUN_EXAMPLES})
    add_executable(rerun_${example} ${example}.cpp)
    target_link_libraries(rerun_${example} PRIVATE nanoPCL::nanoPCL)
    target_compile_definitions(rerun_${example} PRIVATE
      NANOPCL_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
    )
  endforeach()

else()
  message(STATUS "Skipping Rerun examples (USE_RERUN=OFF)")
endif()
