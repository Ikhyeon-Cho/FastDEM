# nanoPCL

<p align="center">
  <img src="https://img.shields.io/github/actions/workflow/status/Ikhyeon-Cho/nanoPCL/cmake.yml?style=flat" alt="CI Status">
  <img src="https://img.shields.io/badge/License-MIT-blue.svg?style=flat" alt="License">
  <img src="https://img.shields.io/badge/C%2B%2B-17-00599C.svg?style=flat" alt="C++17">
  <img src="https://img.shields.io/badge/Header--only-YES-2ea44f.svg?style=flat" alt="Header-only">
  <img src="https://img.shields.io/badge/Dependency-Eigen3-orange.svg?style=flat" alt="Dependency">
</p>

<p align="center">
  <strong>The modern, lightweight alternative to PCL.</strong><br>
  Built for speed. Compile in seconds, not minutes.
</p>

---

## ⚡ Why nanoPCL?

> "PCL is great, but I just need to align two point clouds without installing 2GB of dependencies."

**nanoPCL** is designed for robotics and vision engineers who need performance without the bloat.

| Metric | PCL (Point Cloud Library) | nanoPCL | Improvement |
|:--|:--:|:--:|:--:|
| **Build Time** | Minutes / Hours | **< 5 Seconds** | 🚀 **Instant** |
| **Dependencies** | Boost, FLANN, VTK, QHULL... | **Eigen Only** | 📦 **Zero-Bloat** |
| **Memory (XYZ+I)** | 32 bytes/pt (AoS) | **16 bytes/pt (SoA)** | 💾 **50% Less** |
| **VoxelGrid** | 24.5 ms | **12.1 ms** | ⚡ **2.0x Faster** |
| **Transform** | 18.7 ms | **8.5 ms** | ⚡ **2.2x Faster** |

*(Benchmark: 500k points, Intel Core i7, -O3 optimization)*

---

## 📸 Visuals

<!-- TODO: Insert GIF of Registration or Filtering here -->
> *Visualization powered by Rerun SDK bridge*

---

## 🚀 Quick Start

No installation required. Just copy the header or use CMake `FetchContent`.

### 1. CMake Integration
```cmake
include(FetchContent)
FetchContent_Declare(nanoPCL
  GIT_REPOSITORY https://github.com/Ikhyeon-Cho/nanoPCL.git
  GIT_TAG main)
FetchContent_MakeAvailable(nanoPCL)

target_link_libraries(your_target PRIVATE nanoPCL::nanoPCL)
```

### 2. Code Example
```cpp
#include <nanopcl/common.hpp>

int main() {
    using namespace nanopcl;

    // 1. Load Data (Zero-copy where possible)
    auto cloud = io::loadKITTI("scan.bin");

    // 2. Filter Pipeline (Chainable)
    cloud = filters::cropBox(std::move(cloud), Point(-50,-50,-2), Point(50,50,3));
    cloud = filters::voxelGrid(std::move(cloud), 0.1f);

    // 3. Registration (ICP)
    auto result = registration::alignGICP(source, target);
    
    // 4. Transform
    auto aligned = result.transform(source);
}
```

---

## ✨ Features

- **Core:** SIMD-friendly Structure-of-Arrays (SoA) layout.
- **Filters:** VoxelGrid, CropBox, Outlier Removal (Radius/Statistical).
- **Registration:** ICP, Point-to-Plane, GICP, VGICP (OpenMP accelerated).
- **Search:** VoxelHash (O(1)), KdTree (nanoflann wrapper).
- **Bridge:** Ready-to-use adapters for **ROS 1**, **ROS 2**, **PCL**, and **Rerun**.

---

## 📦 Installation

**Requirements:**
*   C++17 Compiler
*   Eigen3 (≥3.3)

**Optional:**
*   OpenMP (Auto-detected for parallel speedup)
*   Rerun SDK (For visualization examples)

---

## 📜 License

MIT License © [Ikhyeon Cho](mailto:tre0430@korea.ac.kr)