# nanoPCL Examples

Examples are numbered for learning progression. Each example focuses on a specific module.

## Building

```bash
cd nanoPCL
mkdir build && cd build
cmake .. -DNANOPCL_BUILD_EXAMPLES=ON
make
```

## Examples

### Core Module

**01_point_cloud** - PointCloud fundamentals
- Construction (default, capacity, frame_id)
- Adding points (push_back, emplace_back)
- Point access (operator[], at, front, back, data, iterators)
- Container operations (size, capacity, reserve, resize, clear)
- Metadata (frameId, timestamp)
- Merge (operator+=)
- Erase operations
- Copy and move semantics

```bash
./01_point_cloud
```

**02_attributes** - Optional per-point attributes
- Channel lifecycle (enable, disable, check)
- Intensity channel (reflectivity)
- Time channel (for deskewing)
- Ring channel (LiDAR layer index)
- Color channel (RGB)
- Label channel (semantic segmentation)
- Multiple channels together
- Channel behavior with merge/copy

```bash
./02_attributes
```

### Filters Module

**03_filters** - Filtering operations
- Spatial filters (cropBox, cropRange)
- Generic filter with custom predicates
- VoxelGrid filter (downsampling)
- removeInvalid (NaN/Inf removal)

```bash
./03_filters
```

### I/O Module

**04_io** - File I/O
- Save/Load PCD (PCL binary format)
- Save/Load BIN (KITTI format)

```bash
./04_io
```

### Geometry Module

**05_normals** - Surface normal estimation
- PCA-based normal estimation
- Configuration (radius, min_neighbors)
- Performance benchmarking

```bash
./05_normals
```

### Utils Module

**06_deskew** - Motion compensation
- Deskewing with constant velocity model
- Time channel usage
- Before/after comparison

```bash
./06_deskew
```

## Quick Reference

```cpp
#include <nanopcl/nanopcl.hpp>
using namespace nanopcl;

// Create cloud with frame
PointCloud cloud("lidar");

// Add points with attributes
cloud.push_back(Point(x, y, z), Intensity(0.8f), Ring(5));

// Filter (move semantics for optimal performance)
cloud = filters::voxelGrid(std::move(cloud), 0.1f);
cloud = filters::cropBox(std::move(cloud), Point(-5, -5, 0), Point(5, 5, 3));
cloud = filters::cropRange(std::move(cloud), 1.0f, 50.0f);
cloud = filters::removeInvalid(std::move(cloud));

// Generic filter with predicate
cloud = filters::filter(std::move(cloud), [](auto p) {
  return p.point().z() > 0 && p.intensity() > 0.5f;
});

// Transform
SE3d T = SE3d::from2D(x, y, yaw);
cloud = T * cloud;

// I/O
io::loadPCD("input.pcd", cloud);
io::savePCD("output.pcd", cloud);

// Normals
std::vector<Point> normals;
geometry::NormalEstimationConfig ncfg;
ncfg.radius = 0.1f;
geometry::estimateNormals(cloud, normals, ncfg);
```
