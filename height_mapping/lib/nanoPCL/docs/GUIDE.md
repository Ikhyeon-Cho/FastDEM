# nanoPCL User Guide

## Quick Start

```cpp
#include <nanopcl/core.hpp>
using namespace nanopcl;

PointCloud cloud("lidar");
cloud.add(PointXYZI{1, 2, 3, 0.5f});
```

## Core Concepts

### SoA Layout

nanoPCL uses **Structure of Arrays (SoA)** layout instead of PCL's Array of Structures (AoS):

```
PCL (AoS):     [x,y,z,i][x,y,z,i][x,y,z,i]...  ← interleaved
nanoPCL (SoA): [xyz][xyz][xyz]... + [i][i][i]... ← separate vectors
```

**Benefits:**
- Cache-friendly when processing single channels
- SIMD-friendly for vectorization
- Memory-efficient (only allocate channels you use)

### Channels

Available channels (all optional):

| Channel | Type | Enable | Access |
|---------|------|--------|--------|
| XYZ | `Eigen::Vector3f` | Always on | `xyz()` |
| Intensity | `float` | `enableIntensity()` | `intensity()` |
| Time | `float` | `enableTime()` | `time()` |
| Ring | `uint16_t` | `enableRing()` | `ring()` |
| Color | `Color` | `enableColor()` | `color()` |
| Label | `Label` | `enableLabel()` | `label()` |
| Normal | `Eigen::Vector3f` | `enableNormal()` | `normal()` |

## API Patterns

### 1. Adding Points

**DTO Style (Recommended for most cases):**
```cpp
cloud.add(PointXYZI{x, y, z, intensity});
cloud.add(PointXYZIRT{x, y, z, i, ring, time});
```

**Variadic Style (Flexible ordering):**
```cpp
cloud.add(Point(x,y,z), Intensity(i), Ring(r));
cloud.add(Point(x,y,z), Ring(r), Intensity(i));  // order doesn't matter
```

### 2. Accessing Data

**Direct Channel Access (Fast, single channel):**
```cpp
std::vector<float>& intensities = cloud.intensity();
for (float& i : intensities) {
    i *= 2.0f;  // Process all intensities
}
```

**PointRef (Convenient, multiple channels):**
```cpp
for (size_t i = 0; i < cloud.size(); ++i) {
    auto pt = cloud.point(i);
    if (pt.intensity() > 0.5f && pt.ring() < 8) {
        // Use multiple attributes together
    }
}
```

**Iterator (STL compatible):**
```cpp
for (auto pt : cloud) {
    std::cout << pt.x() << ", " << pt.y() << "\n";
}
```

### 3. Filtering

**Use index-based extraction (O(N)), NOT erase (O(N²)):**
```cpp
// GOOD: O(N)
std::vector<size_t> keep;
for (size_t i = 0; i < cloud.size(); ++i) {
    if (cloud[i].z() > 0) keep.push_back(i);
}
PointCloud filtered = cloud[keep];

// BAD: O(N²) - avoid!
for (auto it = cloud.begin(); it != cloud.end();) {
    if (it->z() <= 0) it = cloud.erase(it);  // Slow!
    else ++it;
}
```

## Quick Reference

| Task | Recommended | Why |
|------|-------------|-----|
| Add single point | `add(PointXYZI{...})` | Convenient |
| Bulk insert | `resize() + index` | 2-3x faster |
| Process one channel | `xyz()`, `intensity()` | Cache-friendly |
| Process multiple channels | `point(i)`, iterator | Convenient |
| Filter points | `cloud[indices]` | O(N) vs O(N²) |
| Before bulk add | `reserve(n)` | Avoid reallocations |
| After filtering | `shrink_to_fit()` | Release memory |

## Available DTO Types

| Type | Fields |
|------|--------|
| `PointXYZI` | x, y, z, intensity |
| `PointXYZIR` | x, y, z, intensity, ring |
| `PointXYZIT` | x, y, z, intensity, time |
| `PointXYZIRT` | x, y, z, intensity, ring, time |
| `PointXYZRGB` | x, y, z, r, g, b |
| `PointXYZRGBN` | x, y, z, r, g, b, nx, ny, nz |
| `PointXYZRGBL` | x, y, z, r, g, b, label |
| `PointXYZL` | x, y, z, label |
| `PointXYZIL` | x, y, z, intensity, label |
| `PointXYZN` | x, y, z, nx, ny, nz |
| `PointXYZIN` | x, y, z, intensity, nx, ny, nz |

## See Also

- `examples/` - Usage examples
- `examples/recipes/` - Advanced patterns
- `docs/performance.md` - Performance optimization guide
