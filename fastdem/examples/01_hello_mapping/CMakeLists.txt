add_executable(01_hello_mapping main.cpp)
target_link_libraries(01_hello_mapping PRIVATE fastdem)
target_compile_definitions(01_hello_mapping PRIVATE
  EXAMPLE_OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
