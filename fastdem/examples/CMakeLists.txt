# height_mapping examples

# Common header-only utilities
add_library(example_common INTERFACE)
target_include_directories(example_common INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/common
)

# Basic examples
add_subdirectory(01_hello_mapping)
add_subdirectory(02_config_loading)
add_subdirectory(03_estimator_comparison)
add_subdirectory(04_transform_provider)
