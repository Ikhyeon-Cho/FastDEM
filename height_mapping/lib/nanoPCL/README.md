# nanoPCL

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Header-only](https://img.shields.io/badge/header--only-yes-green.svg)]()

**Lightweight point cloud library. No ROS. No PCL. Just Eigen.**

```cpp
#include <nanopcl/nanopcl.hpp>
using namespace nanopcl;

PointCloud cloud("lidar");
cloud.push_back(Point(1, 2, 3));

cloud = filters::voxelGrid(cloud, 0.1f);  // Downsample
SE3d T = SE3d::from2D(1.0, 0.0, M_PI/4);  // Transform
cloud = T * cloud;
```

## Installation

```cmake
# CMakeLists.txt
include(FetchContent)
FetchContent_Declare(nanoPCL
  GIT_REPOSITORY https://github.com/Ikhyeon-Cho/nanoPCL.git
  GIT_TAG main)
FetchContent_MakeAvailable(nanoPCL)
target_link_libraries(your_target PRIVATE nanoPCL::nanoPCL)
```

**Requirements:** C++17, Eigen3 (>= 3.3)

## Features

| Module | Functions |
|--------|-----------|
| **Core** | `PointCloud`, `Point`, timestamps, frame IDs |
| **Filters** | `voxelGrid`, `cropBox`, `cropRange`, `filter`, `removeInvalid` |
| **Transforms** | `SE3`, `SO3`, frame-aware `Transform` |
| **Geometry** | `AABB`, `OBB`, `estimateNormals` |
| **I/O** | PCD, BIN (KITTI), TUM trajectory |

## Examples

See [`examples/`](examples/) for working code:

- **01_point_cloud** - Core PointCloud API, memory management
- **02_attributes** - Optional channels (intensity, time, ring, color, label)
- **03_filters** - VoxelGrid, Passthrough filtering
- **04_io** - Load/save PCD, BIN, TUM formats
- **05_normals** - Surface normal estimation
- **06_deskew** - Motion compensation

## Performance

Benchmark: 500k points, Release build (`-O3`), Intel Core i7

| Operation | nanoPCL | PCL | Speedup |
|-----------|---------|-----|---------|
| Construction | 0.02 ms | 7.8 ms | **350x** |
| Transform | 8.5 ms | 18.7 ms | **2.2x** |
| Random Access | 4.5 ms | 8.2 ms | **1.8x** |
| Deep Copy | 4.5 ms | 7.3 ms | **1.6x** |

**Memory per point:**
| Type | nanoPCL | PCL |
|------|---------|-----|
| XYZ only | 12 bytes | 16 bytes (PointXYZ) |
| XYZ + Intensity | 16 bytes | 32 bytes (PointXYZI) |
| Full (XYZ+I+Ring) | 18 bytes | 48 bytes (PointXYZINormal) |

### Why is nanoPCL faster?

**Structure of Arrays (SoA)** vs PCL's Array of Structures (AoS):

```cpp
// nanoPCL (SoA) - Contiguous XYZ, optional attributes
points_:     [x0,y0,z0] [x1,y1,z1] [x2,y2,z2] ...  // Cache-friendly for transforms
intensity_:  [i0] [i1] [i2] ...                     // Allocated only if needed

// PCL (AoS) - Interleaved, always allocated
points_:     [x0,y0,z0,pad,i0,pad,pad,pad] [x1,y1,z1,pad,i1,pad,pad,pad] ...
```

Run benchmarks: `cmake -DNANOPCL_BUILD_BENCHMARKS=ON .. && make && ./benchmarks/benchmark_pointcloud`

## Why nanoPCL?

| | PCL | nanoPCL |
|--|-----|---------|
| Build time | Minutes | Seconds |
| Dependencies | VTK, Boost, FLANN... | Eigen only |
| Binary size | Large | Zero (header-only) |
| Memory | Fixed per point type | Pay for what you use |

nanoPCL is **not** a PCL replacement. It's for when you need basic operations without heavyweight dependencies.

## License

MIT License - [Ikhyeon Cho](mailto:tre0430@korea.ac.kr), Korea University
