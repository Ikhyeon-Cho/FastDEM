# nanoPCL Examples

Examples are numbered for learning progression.

## Building

```bash
cd nanoPCL
mkdir build && cd build
cmake .. -DNANOPCL_BUILD_EXAMPLES=ON
make
```

## Examples

### 01_hello_world - Getting Started
Core PointCloud usage in one file.
- Create cloud, add points (PointXYZI)
- Index-based and range-based iteration
- Direct channel access
- Fancy indexing (subset extraction)
- Frame and timestamp metadata

```bash
./01_hello_world
```

### 02_point_types - PCL-style Point Types
Composite point types for convenient insertion.
- LiDAR: PointXYZI, PointXYZIR, PointXYZIT, PointXYZIRT
- RGB: PointXYZRGB, PointXYZRGBN, PointXYZRGBL
- Segmentation: PointXYZL, PointXYZIL
- Normal: PointXYZN, PointXYZIN
- Variadic API alternative

```bash
./02_point_types
```

### 03_channels - Channel System
Optional per-point attribute channels.
- Enable/disable channels (memory efficiency)
- Auto-enable behavior
- Direct channel access (fast, single-channel ops)
- PointRef access (convenient, multi-channel logic)

```bash
./03_channels
```

### 04_indexing - Indexing & Operations
Point cloud manipulation.
- Basic indexing ([], at, front, back)
- Fancy indexing (subset extraction)
- Container operations (reserve, clear, shrink_to_fit)
- Merge operations (+=, +)

```bash
./04_indexing
```

### 05_io - File I/O
Save and load point clouds.
- PCD format (PCL compatible)
- BIN format (KITTI compatible)

```bash
./05_io
```

### 06_filtering - Filters
Point cloud filtering operations.
- cropBox (axis-aligned bounding box)
- cropRange (distance-based, ring shape)
- voxelGrid (downsampling)
- Custom predicates (full flexibility)
- Copy vs Move semantics
- Filter chaining (pipeline pattern)

```bash
./06_filtering
```

### 07_transform - Transform Basics
Rigid body transformations with frame awareness.
- Creating transforms (x, y, yaw / RPY / matrix)
- Accessing components (translation, rotation, quaternion)
- Transform composition (frame-checked)
- Inverse transform
- Interpolation (slerp)

```bash
./07_transform
```

### 08_transform_cloud - Transforming Point Clouds
Apply transforms to point clouds.
- Basic transformation (frame auto-update)
- Frame validation (mismatch detection)
- Copy vs Move vs InPlace
- Transform chain
- Normal vector handling

```bash
./08_transform_cloud
```

### 09_normals - Normal Estimation
Surface normal estimation using PCA.
- Plane and sphere normal verification
- Radius parameter tuning
- min_neighbors parameter
- VoxelHash-based neighbor search

```bash
./09_normals
```

### 10_search - Spatial Search
Spatial indexing for neighbor queries.
- VoxelHash (O(1) radius search)
- KdTree (O(log N) kNN search)
- Callback vs vector API comparison

```bash
./10_search
```

### 11_clustering - Euclidean Clustering
Point cloud segmentation for object detection.
- Ground removal (RANSAC plane)
- Euclidean clustering with VoxelHash
- Cluster extraction and labeling
- CSR-based direct access

```bash
./11_clustering
```

### 12_registration - Point Cloud Registration
Align source cloud to target using ICP variants.
- Point-to-Point ICP (baseline)
- Point-to-Plane ICP (requires normals)
- GICP (requires covariances)
- VGICP (fastest, voxelized target)
- Accuracy vs speed comparison

```bash
./12_registration
```

## Recipes (Advanced)

Located in `recipes/` directory:
- **normals** - Surface normal estimation
- **deskew** - Motion compensation for spinning LiDAR
- **performance_patterns** - Optimization techniques

## Quick Reference

```cpp
#include <nanopcl/common.hpp>
using namespace npcl;

// Create and populate
PointCloud cloud("lidar");
cloud.add(PointXYZI{x, y, z, intensity});
cloud.add(Point(x, y, z), Intensity(0.8f), Ring(5));

// Filter pipeline (move for efficiency)
cloud = filters::cropRange(std::move(cloud), 1.0f, 50.0f);
cloud = filters::cropBox(std::move(cloud), min_pt, max_pt);
cloud = filters::voxelGrid(std::move(cloud), 0.1f);

// Custom filter
cloud = filters::filter(std::move(cloud), [](auto p) {
  return p.point().z() > 0 && p.intensity() > 0.5f;
});

// Transform
Transformd T("lidar", "base_link");
T.setTranslation(0, 0, 1);
cloud = transformCloud(std::move(cloud), T);

// I/O
io::savePCD("output.pcd", cloud);
io::loadPCD("input.pcd", cloud);
```
