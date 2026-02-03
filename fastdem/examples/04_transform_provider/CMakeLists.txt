add_executable(04_transform_provider main.cpp)
target_link_libraries(04_transform_provider PRIVATE fastdem)
target_compile_definitions(04_transform_provider PRIVATE
  EXAMPLE_OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
