# nanoPCL

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Header-only](https://img.shields.io/badge/header--only-yes-green.svg)]()

**High-performance, lightweight point cloud library for robotics. No ROS. No PCL. Just Eigen.**

```cpp
#include <nanopcl/common.hpp>
using namespace npcl;

PointCloud cloud("lidar");
cloud.add(Point(1, 2, 3), Intensity(0.5f), Ring(5)); // Variadic API

// Optimized filtering (1.8x faster than PCL via SoA)
cloud = filters::voxelGrid(std::move(cloud), 0.1f);

// High-speed registration (Fastest GICP/VGICP implementation)
registration::VoxelDistributionMap voxel_map(0.5f);
voxel_map.build(target);
auto result = registration::vgicp(source, voxel_map);
```

## 🚀 Key Differentiators

*   **Structure of Arrays (SoA)**: Unlike PCL's AoS, nanoPCL separates attributes (Intensity, Time, etc.) into contiguous arrays. This maximizes cache locality for XYZ-only operations (e.g., transforms) and SIMD vectorization.
*   **Zero-Overhead Abstractions**: Access SoA data as if they were structs using `PointRef` proxies. Use `add(PointXYZI{...})` for PCL compatibility or `add(p, Intensity(i))` for maximum flexibility.
*   **Optimized Spatial Hashing**: Implements Teschner et al.'s spatial hashing for $O(1)$ radius search, providing significantly faster index building than KD-Trees for real-time applications.
*   **State-of-the-Art Registration**: Includes **VGICP (Voxelized GICP)**, delivering the accuracy of GICP at the speed of voxel-based methods.

## 🛠 Features

| Module | Key Functions & Components |
|--------|----------------------------|
| **Core** | `PointCloud` (SoA), `PointRef`, `Label` (32-bit packed), Frame-aware `Transform` |
| **Filters** | `voxelGrid` (Bit-packed), `cropBox`, `cropAngle` (atan2-free), Outlier Removal (ROR/SOR) |
| **Search** | `VoxelHash` (Teschner Hashing), `KdTree` (nanoflann wrapper) |
| **Geometry** | PCA-based `estimateNormals`, Local Covariance Estimation |
| **Registration**| `icp`, `icpPlane`, **GICP**, **VGICP** (Voxel-based Target Distributions) |
| **Bridge** | **ROS 1/2** (`sensor_msgs/PointCloud2`), **PCL** (`pcl::PointCloud<T>`) |

## 📖 Examples

Comprehensive tutorials located in [`examples/`](examples/):

- **[01-04] Core**: Cloud creation, Strong types, Indexing, and PointRef usage.
- **[06] Filtering**: Spatial cropping, VoxelGrid downsampling, and Move semantics.
- **[07-08] Transform**: Frame-aware TF2-style transformations and point cloud deskewing.
- **[10] Search**: `VoxelHash` vs `KdTree` performance comparison.
- **[11] Segmentation**: Ground removal (RANSAC) and Euclidean Clustering (CSR format).
- **[12] Registration**: Step-by-step guide from basic ICP to high-speed **VGICP**.

## 📊 Performance

Benchmark: 500k points, Release build (`-O3`), Intel Core i7

| Operation | nanoPCL | PCL | Speedup |
|-----------|---------|-----|---------|
| Construction | 0.02 ms | 7.8 ms | **350x** |
| Transform | 8.5 ms | 18.7 ms | **2.2x** |
| VoxelGrid (0.1m) | 12.1 ms | 24.5 ms | **2.0x** |
| Deep Copy | 4.5 ms | 7.3 ms | **1.6x** |

**Memory Efficiency (Pay-for-what-you-use):**
| Channels | nanoPCL (SoA) | PCL (AoS) | Saved |
|----------|---------------|-----------|-------|
| XYZ only | 12 bytes | 16 bytes | 25% |
| XYZ + I | 16 bytes | 32 bytes | 50% |
| XYZ + I + R | 18 bytes | 48 bytes | 62% |

## ⚙️ Installation

```cmake
# Add to your CMakeLists.txt
include(FetchContent)
FetchContent_Declare(nanoPCL
  GIT_REPOSITORY https://github.com/Ikhyeon Cho/nanoPCL.git
  GIT_TAG main)
FetchContent_MakeAvailable(nanoPCL)
target_link_libraries(your_target PRIVATE nanoPCL::nanoPCL)
```

**Requirements:** C++17, Eigen3 (>= 3.3). OpenMP is highly recommended for parallel acceleration.

## 📜 License

MIT License - [Ikhyeon Cho](mailto:tre0430@korea.ac.kr), Korea University.
"No ROS. No PCL. Just Eigen."
