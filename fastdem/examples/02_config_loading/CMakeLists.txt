add_executable(02_config_loading main.cpp)
target_link_libraries(02_config_loading PRIVATE fastdem)
target_compile_definitions(02_config_loading PRIVATE
  EXAMPLE_OUTPUT_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  EXAMPLE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../config"
)
