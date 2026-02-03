# nanoPCL Tests

# Core v2 tests
add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_types COMMAND test_types)

add_executable(test_pointcloud test_pointcloud.cpp)
target_link_libraries(test_pointcloud PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_pointcloud COMMAND test_pointcloud)

add_executable(test_channels test_channels.cpp)
target_link_libraries(test_channels PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_channels COMMAND test_channels)

# Filters module tests
add_executable(test_filters test_filters.cpp)
target_link_libraries(test_filters PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_filters COMMAND test_filters)

# Search module tests
add_executable(test_search test_search.cpp)
target_link_libraries(test_search PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_search COMMAND test_search)

# Voxel utilities tests
add_executable(test_voxel test_voxel.cpp)
target_link_libraries(test_voxel PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_voxel COMMAND test_voxel)

# Geometry module tests
add_executable(test_geometry test_geometry.cpp)
target_link_libraries(test_geometry PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_geometry COMMAND test_geometry)

# IndexRange tests
add_executable(test_index_range test_index_range.cpp)
target_link_libraries(test_index_range PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_index_range COMMAND test_index_range)

# Transform tests
add_executable(test_transform test_transform.cpp)
target_link_libraries(test_transform PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_transform COMMAND test_transform)

# Registration module tests
add_executable(test_registration test_registration.cpp)
target_link_libraries(test_registration PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_registration COMMAND test_registration)

# IO module tests
add_executable(test_io test_io.cpp)
target_link_libraries(test_io PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_io COMMAND test_io)

# Segmentation module tests
add_executable(test_segmentation test_segmentation.cpp)
target_link_libraries(test_segmentation PRIVATE nanoPCL::nanoPCL)
add_test(NAME test_segmentation COMMAND test_segmentation)
