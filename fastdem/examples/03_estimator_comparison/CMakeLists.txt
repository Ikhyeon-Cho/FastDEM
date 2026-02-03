add_executable(03_estimator_comparison main.cpp)
target_link_libraries(03_estimator_comparison PRIVATE fastdem)
target_compile_definitions(03_estimator_comparison PRIVATE
  EXAMPLE_OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
