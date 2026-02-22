include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0
)

# Prevent overriding parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Helper function
function(fastdem_add_test TEST_NAME)
  add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
  target_link_libraries(${TEST_NAME} PRIVATE fastdem GTest::gtest_main)
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

# Config test needs to know where YAML files are
add_executable(test_config test_config.cpp)
target_link_libraries(test_config PRIVATE fastdem GTest::gtest_main)
target_compile_definitions(test_config PRIVATE
  FASTDEM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../config")
add_test(NAME test_config COMMAND test_config)

# Test targets
fastdem_add_test(test_elevation_map)
fastdem_add_test(test_rasterization)
fastdem_add_test(test_kalman_estimation)
fastdem_add_test(test_quantile_estimation)
fastdem_add_test(test_dual_layer)
fastdem_add_test(test_sensor_models)
fastdem_add_test(test_postprocess)
fastdem_add_test(test_fastdem_integration)
fastdem_add_test(test_online_mode)
fastdem_add_test(test_map_io)
