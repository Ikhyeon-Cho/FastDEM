# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build examples
mkdir build && cd build
cmake .. -DNANOPCL_BUILD_EXAMPLES=ON
make

# Build benchmarks (requires PCL: sudo apt install libpcl-dev)
cmake .. -DNANOPCL_BUILD_BENCHMARKS=ON
make

# Run examples from build directory
./01_hello_world
./06_filtering
```

**Requirements:** C++17, Eigen3 >= 3.3, OpenMP (optional)

## Architecture Overview

nanoPCL is a **header-only** point cloud library using **Structure of Arrays (SoA)** layout for cache-efficient operations. Coordinates and attributes are stored in separate contiguous arrays (vs PCL's interleaved AoS).

### Module Structure

| Header | Contents |
|--------|----------|
| `common.hpp` | Core + Transform + Filters (default include) |
| `io.hpp` | PCD, BIN (KITTI), TUM trajectory I/O |
| `search.hpp` | VoxelHash (O(1)), KdTree (O(log N)) |
| `registration.hpp` | ICP, Point-to-Plane ICP, GICP, VGICP |
| `segmentation.hpp` | Euclidean clustering, RANSAC plane |
| `geometry/local_surface.hpp` | Normal/covariance estimation |
| `bridge/ros1.hpp` | ROS 1 sensor_msgs conversion |
| `bridge/ros2.hpp` | ROS 2 sensor_msgs conversion |
| `bridge/pcl.hpp` | PCL pcl::PointCloud conversion |
| `transform/lie.hpp` | SE(3)/SO(3) exponential maps |

### Key Design Patterns

**1. SoA Memory Layout**
```cpp
points_:     [x0,y0,z0] [x1,y1,z1] ...  // Cache-friendly for transforms
intensity_:  [i0] [i1] ...               // Only allocated when enabled
```

**2. Lazy Channel Allocation**
```cpp
cloud.enableIntensity();  // Allocates intensity_ vector
cloud.hasIntensity();     // Check without allocation
```

**3. Move Semantics for Pipelines**
```cpp
cloud = filters::cropRange(std::move(cloud), 1.0f, 50.0f);
cloud = filters::voxelGrid(std::move(cloud), 0.1f);
```

**4. Frame-Aware Transforms (TF2 semantics)**
```cpp
Transform T("map", "base_link");  // T_map_base_link (parent, child)
cloud = transformCloud(cloud, T); // Frame validation on composition
```

### Core APIs

| Component | Key Functions |
|-----------|---------------|
| `PointCloud` | `add()`, `xyz()`, `intensity()`, `time()`, `ring()`, `normal()`, `covariance()` |
| `filters::` | `voxelGrid()`, `cropBox()`, `cropRange()`, `cropX/Y/Z()`, `cropAngle()`, `filter()` |
| `Transform` | `fromRPY()`, `from2D()`, `inverse()`, `slerp()`, `operator*` |
| `registration::` | `icp()`, `icpPlane()`, `gicp()`, `vgicp()` |
| `segmentation::` | `euclideanCluster()`, `segmentPlane()` |
| `geometry::` | `estimateNormals()`, `estimateCovariances()`, `estimateLocalSurface()` |
| `io::` | `loadPCD()`, `savePCD()`, `loadBIN()`, `saveBIN()` |

### ROS Bridge Usage
```cpp
#include <nanopcl/bridge/ros1.hpp>

void callback(const sensor_msgs::PointCloud2& msg) {
    auto cloud = npcl::from(msg);
    // ... process ...
    auto out = npcl::to<sensor_msgs::PointCloud2>(cloud);
}

// Transform conversion
auto tf = npcl::from(tf_stamped_msg);
```

### Registration Example
```cpp
// Point-to-Point ICP
auto result = registration::icp(source, target);

// VGICP (fastest)
geometry::estimateCovariances(source, 0.3f);
registration::VoxelDistributionMap voxel_map(0.5f);
voxel_map.build(target);
auto result = registration::vgicp(source, voxel_map);
```

## Performance Notes

- Per-point transform loop is 5x faster than batch GEMM (see benchmarks/README.md)
- SoA layout provides 2-350x speedup over PCL depending on operation
- Memory: 12 bytes/point (XYZ) vs PCL's 16+ bytes
- ICP: 13-28x faster than PCL (VoxelHash + nanoflann KdTree)

## Third-Party Dependencies

- **nanoflann**: Header-only KD-tree in `thirdparty/` (for KdTree search)
- **Eigen3**: Required for linear algebra
- **OpenMP**: Optional for parallel normal estimation and ICP
