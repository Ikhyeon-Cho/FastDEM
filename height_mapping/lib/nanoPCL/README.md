# nanoPCL

**Header-only C++17 point cloud library. No ROS. No PCL. Just Eigen.**

PCL is powerful but heavy. nanoPCL gives you the most-used features in a single header.

## Features

- **Point Cloud** - Container with frame/timestamp metadata
- **Filters** - VoxelGrid, Passthrough
- **Transforms** - SE(3)/SO(3) with frame-aware operations
- **Geometry** - AABB, OBB bounding boxes

## Quick Start

```cpp
#include <nanopcl/nanopcl.hpp>

using namespace nanopcl;

// Create point cloud
PointCloud cloud("lidar");
cloud.push_back(Point(1.0f, 2.0f, 3.0f));
// ... add more points

// Downsample
filters::VoxelGrid voxel({.voxel_size = 0.1f});
voxel.filterInPlace(cloud);

// Filter by bounds
filters::Passthrough pass({.z_min = -1.0f, .z_max = 2.0f});
pass.filterInPlace(cloud);

// Transform
SE3d T = SE3d::from2D(1.0, 0.0, M_PI/4);
cloud = T * cloud;
```

## Installation

### CMake FetchContent (Recommended)

```cmake
include(FetchContent)
FetchContent_Declare(
  nanoPCL
  GIT_REPOSITORY https://github.com/Ikhyeon-Cho/nanoPCL.git
  GIT_TAG main
)
FetchContent_MakeAvailable(nanoPCL)

target_link_libraries(your_target PRIVATE nanoPCL::nanoPCL)
```

### Manual

```bash
git clone https://github.com/Ikhyeon-Cho/nanoPCL.git
cd nanoPCL
mkdir build && cd build
cmake ..
sudo make install
```

## Requirements

- C++17 compiler
- Eigen3 (>= 3.3)

## API Reference

### Core Types

```cpp
// Point (Eigen::Vector3f alias)
Point p(1.0f, 2.0f, 3.0f);
float dist = distance(p, Point::Zero());

// PointCloud with metadata
PointCloud cloud("laser_frame");
cloud.setTimestamp(timestamp_ns);
cloud.push_back(Point(x, y, z));
for (const auto& pt : cloud) { /* ... */ }
```

### Filters

```cpp
// VoxelGrid - downsample point cloud
filters::VoxelGrid voxel({
  .voxel_size = 0.1f,
  .method = filters::VoxelGrid::Method::CENTROID
});
auto stats = voxel.filterInPlace(cloud);

// Passthrough - filter by bounds
filters::Passthrough pass({
  .x_min = -10.0f, .x_max = 10.0f,
  .z_min = -1.0f, .z_max = 3.0f,
  .use_distance_filter = true,
  .distance_max = 50.0f
});
pass.filterInPlace(cloud);
```

### Transforms

```cpp
// SE3 rigid body transform
SE3d T = SE3d::from2D(x, y, yaw);
SE3d T = SE3d::fromRPY(roll, pitch, yaw);
SE3d T = SE3d::fromQuaternion(qx, qy, qz, qw);

// Transform point cloud
PointCloud cloud_map = T * cloud_lidar;

// Frame-aware Transform
Transform tf;
tf.from("lidar").to("map").with(se3);
PointCloud cloud_map = tf * cloud_lidar;  // validates frames

// Composition with frame checking
Transform T_map_base = ...;
Transform T_base_lidar = ...;
Transform T_map_lidar = T_map_base * T_base_lidar;  // throws if frames don't match
```

### Geometry

```cpp
// Axis-Aligned Bounding Box
AABB box = AABB::fromCenterSize(center, size, "map");
bool inside = box.contains(point);

// Oriented Bounding Box
OBB obb(center, orientation, extent, "map");
AABB aabb = obb.toAABB();
```

## Comparison with PCL

| Feature | PCL | nanoPCL |
|---------|-----|---------|
| Build time | Minutes | Seconds |
| Binary size | Large | Tiny |
| Dependencies | Many (VTK, Boost, ...) | Eigen only |
| VoxelGrid filter | ✓ | ✓ |
| Passthrough filter | ✓ | ✓ |
| ICP/NDT | ✓ | ✗ |
| Visualization | ✓ | ✗ |
| PCD I/O | ✓ | ✗ (planned) |

nanoPCL is **not** a PCL replacement. It's for when you need basic point cloud operations without the heavyweight dependency.

## License

MIT License - see [LICENSE](LICENSE) for details.

## Author

Ikhyeon Cho (tre0430@korea.ac.kr)
Korea University, Intelligent Systems & Robotics Lab
