# Registration Module

Point cloud registration algorithms for nanoPCL.

## Quick Start

```cpp
#include <nanopcl/registration.hpp>
using namespace nanopcl;

// Point-to-Point ICP
auto result = registration::icp(source, target);

// Point-to-Plane ICP (requires normals)
geometry::estimateNormals(target, 0.3f);
auto result = registration::icpPlane(source, target);

// GICP (requires covariances)
geometry::estimateCovariances(source, 0.3f);
geometry::estimateCovariances(target, 0.3f);
auto result = registration::gicp(source, target);

// VGICP (fastest for repeated registrations)
geometry::estimateCovariances(source, 0.3f);
registration::VoxelDistributionMap voxel_map(0.5f);
voxel_map.build(target);
auto result = registration::vgicp(source, voxel_map);
```

## Algorithms

| Algorithm | Error Metric | Correspondence | Use Case |
|-----------|--------------|----------------|----------|
| **ICP** | Point-to-Point | k-d tree O(log N) | Simple, baseline |
| **ICP-Plane** | Point-to-Plane | k-d tree O(log N) | Fast convergence |
| **GICP** | Plane-to-Plane | k-d tree O(log N) | Highest accuracy |
| **VGICP** | Plane-to-Plane | Voxel O(1) | Speed + accuracy |

## API Reference

### RegistrationResult

```cpp
struct RegistrationResult {
  Eigen::Isometry3d transform;  // T_target_source
  double fitness;               // Ratio of valid correspondences [0, 1]
  double rmse;                  // Root mean square error
  size_t iterations;            // Number of iterations performed
  bool converged;               // Whether algorithm converged
};
```

### ICP (Point-to-Point)

```cpp
struct ICPConfig {
  int max_iterations = 50;
  float max_correspondence_dist = 1.0f;
  size_t min_correspondences = 10;
  double translation_threshold = 1e-6;
  double rotation_threshold = 1e-6;
  double relative_mse_threshold = 1e-6;
};

// With pre-built search index (recommended for repeated registrations)
template <typename SearchMethod>
RegistrationResult icp(const PointCloud& source,
                       const PointCloud& target,
                       const SearchMethod& target_search,
                       const Isometry3d& initial_guess = Identity(),
                       const ICPConfig& config = {});

// Builds search index internally
template <typename SearchMethod = search::KdTree>
RegistrationResult icp(const PointCloud& source,
                       const PointCloud& target,
                       const Isometry3d& initial_guess = Identity(),
                       const ICPConfig& config = {});
```

### ICP-Plane (Point-to-Plane)

```cpp
// Requires: target must have normals
// Call geometry::estimateNormals(target, radius) first

RegistrationResult icpPlane(const PointCloud& source,
                            const PointCloud& target,
                            const Isometry3d& initial_guess = Identity(),
                            const ICPPlaneConfig& config = {});
```

### GICP (Generalized ICP)

```cpp
struct GICPConfig : ICPConfig {
  double covariance_epsilon = 1e-3;  // Regularization parameter
};

// Requires: both source and target must have covariances
// Call geometry::estimateCovariances(cloud, radius) first

RegistrationResult gicp(const PointCloud& source,
                        const PointCloud& target,
                        const Isometry3d& initial_guess = Identity(),
                        const GICPConfig& config = {});
```

### VGICP (Voxelized GICP)

```cpp
struct VGICPConfig {
  int max_iterations = 50;
  size_t min_correspondences = 10;
  double translation_threshold = 1e-6;
  double rotation_threshold = 1e-6;
  double relative_mse_threshold = 1e-6;
  double covariance_epsilon = 1e-3;
};

// With pre-built voxel map (recommended)
RegistrationResult vgicp(const PointCloud& source,
                         const VoxelDistributionMap& voxel_map,
                         const Isometry3d& initial_guess = Identity(),
                         const VGICPConfig& config = {});

// Builds voxel map internally
RegistrationResult vgicp(const PointCloud& source,
                         const PointCloud& target,
                         float voxel_resolution,
                         const Isometry3d& initial_guess = Identity(),
                         const VGICPConfig& config = {});
```

### VoxelDistributionMap

```cpp
class VoxelDistributionMap {
public:
  explicit VoxelDistributionMap(float resolution = 0.5f,
                                 double covariance_epsilon = 1e-3);

  void build(const PointCloud& cloud);  // O(N), parallelized

  // O(1) lookup
  std::optional<VoxelDistribution> lookup(const Point& p) const;
  std::optional<VoxelDistribution> lookupRegularized(const Point& p) const;

  float resolution() const;
  size_t numVoxels() const;
  bool empty() const;
};
```

## Usage Patterns

### Single Registration

```cpp
auto result = registration::icp(source, target);
if (result.converged) {
  PointCloud aligned = result.transform * source;
}
```

### Repeated Registration (SLAM)

```cpp
// Build search index once
search::KdTree tree;
tree.build(target);

// Register multiple scans
for (const auto& scan : scans) {
  auto result = registration::icp(scan, target, tree, initial_guess);
  // ...
}
```

### VGICP for Localization

```cpp
// Build voxel map from map cloud (one-time)
registration::VoxelDistributionMap map(0.5f);
map.build(map_cloud);

// Localize incoming scans
for (const auto& scan : scans) {
  geometry::estimateCovariances(scan, 0.3f);
  auto result = registration::vgicp(scan, map, odometry_guess);
  // ...
}
```

## Performance Comparison

Benchmark: 50,000 points, 8 threads, Release build

| Algorithm | Time/iter | Total (20 iter) | Notes |
|-----------|-----------|-----------------|-------|
| ICP | ~1.1 ms | ~22 ms | Baseline |
| ICP-Plane | ~1.2 ms | ~12 ms | Fewer iterations needed |
| GICP | ~2.5 ms | ~12 ms | Higher accuracy |
| VGICP | ~0.8 ms | ~8 ms | O(1) lookup advantage |

## Implementation Details

### Optimizations

- **OpenMP parallelization**: Correspondence search and solver
- **Upper-triangular Hessian**: 21 elements instead of 36
- **Thread-local buffers**: Lock-free parallel reduction
- **Analytical 3x3 inverse**: Cramer's rule (faster than Eigen)
- **Pre-computed covariances**: Regularization done once before iteration loop

### Convergence Criteria

Registration converges when ANY of these conditions are met:

1. **Transform stable**: `‖Δt‖ < threshold` AND `‖Δθ‖ < threshold`
2. **MSE stalled**: `|MSE_prev - MSE| / MSE_prev < threshold`

### References

- Besl & McKay (1992) - ICP
- Chen & Medioni (1992) - Point-to-Plane ICP
- Segal, Haehnel, Thrun (RSS 2009) - Generalized ICP
- Koide et al. (ICRA 2021) - Voxelized GICP
