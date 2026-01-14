# nanoPCL Developer Guide

## 1. Project Overview
**nanoPCL** is a high-performance, header-only C++17 library for point cloud processing. It serves as a lightweight alternative to PCL (Point Cloud Library), specifically optimized for real-time robotics and LiDAR applications.

**Key Differentiator:** "Structure of Arrays" (SoA) memory layout vs PCL's "Array of Structures" (AoS).
**Motto:** "No ROS. No PCL. Just Eigen."

## 2. Architecture & Design

### 2.1 Memory Layout (SoA)
Unlike PCL which stores points as structs (e.g., `struct PointXYZI { float x,y,z,intensity; }`), nanoPCL separates data into contiguous arrays:
```cpp
class PointCloud {
  std::vector<Point> points_;       // [x,y,z], [x,y,z], ...
  std::vector<float> intensity_;    // [i], [i], ... (Optional)
  std::vector<double> time_;        // [t], [t], ... (Optional)
  // ...
};
```
**Benefits:**
*   **Cache Locality:** Operations on coordinates (e.g., transforms) don't load unused attribute data into cache lines.
*   **SIMD Friendly:** Contiguous floats allow for better vectorization.
*   **Memory Efficiency:** Channels like `intensity` or `ring` consume 0 bytes if not used.

### 2.2 The `PointRef` Proxy
To maintain a user-friendly API, nanoPCL uses a proxy pattern:
```cpp
// "pt" is not a struct, but a lightweight proxy object
for (auto pt : cloud) {
    pt.z() += 1.0;          // Modifies points_[i].z
    if (pt.hasIntensity())  // Checks intensity_ pointer
        pt.intensity() *= 2;// Modifies intensity_[i]
}
```

## 3. Core Modules

### 3.1 Search Strategies
Two distinct implementations cover different use cases:

| Feature | **VoxelHash** (`search/voxel_hash.hpp`) | **KdTree** (`search/kdtree.hpp`) |
| :--- | :--- | :--- |
| **Algorithm** | Spatial Hashing (Flat Hash Map) | KD-Tree (via `nanoflann`) |
| **Build Time** | O(N) - Very Fast | O(N log N) |
| **Query Time** | O(1) - Radius Search | O(log N) - KNN |
| **Best For** | Real-time LiDAR, Radius queries | ICP, Feature matching, KNN |
| **Optimization**| "Layer 0-1" check for fast nearest neighbor | Standard tree traversal |

### 3.2 Filtering (`filters/voxel_grid.hpp`)
Highly optimized downsampling implementation:
*   **Bit Packing:** Voxel coordinates are packed into a single `uint64_t` for fast sorting.
*   **TLS Optimization:** Uses Thread Local Storage buffers to avoid allocations during repeated calls (critical for high-frequency loops).
*   **Methods:** `CENTROID` (Average), `FIRST`, `CLOSEST` (to center), `ARBITRARY`.

### 3.3 Transforms & Frame Safety (`transform/transform.hpp`)
The `Transform` class is frame-aware to prevent robotics math errors:
```cpp
Transform T_map_base("map", "base_link");
Transform T_base_lidar("base_link", "lidar");

// OK: "map" -> "base_link" -> "lidar"
Transform T_map_lidar = T_map_base * T_base_lidar; 

// Error: Frame mismatch throws std::runtime_error
Transform invalid = T_base_lidar * T_map_base;
```

## 4. Build & Integration

### Requirements
*   **C++17**
*   **Eigen3** (>= 3.3)
*   **OpenMP** (Optional, for parallel ops)

### CMake Integration
```cmake
include(FetchContent)
FetchContent_Declare(nanoPCL
  GIT_REPOSITORY https://github.com/Ikhyeon-Cho/nanoPCL.git
  GIT_TAG main)
FetchContent_MakeAvailable(nanoPCL)
target_link_libraries(your_target PRIVATE nanoPCL::nanoPCL)
```

## 5. Performance Guidelines
1.  **Prefer Move Semantics:**
    ```cpp
    // Good: Reuses memory buffers
    cloud = filters::voxelGrid(std::move(cloud), 0.1f);
    ```
2.  **Loop Fission:** When processing points, writing separate loops for XYZ and Intensity is faster than one combined loop due to cache locality.
3.  **Pre-allocation:** Always call `cloud.reserve(N)` if size is known.
4.  **Vector Access:** For maximum speed, access raw vectors directly:
    ```cpp
    float* intensities = cloud.intensity().data();
    for(size_t i=0; i<n; ++i) intensities[i] *= 2.0f;
    ```

## 6. Development Tools
*   **Benchmarks:** Run `./build/benchmarks/benchmark_pointcloud` to verify performance on your hardware.
*   **Examples:** See `examples/` for reference implementations of I/O, filtering, and transforms.
