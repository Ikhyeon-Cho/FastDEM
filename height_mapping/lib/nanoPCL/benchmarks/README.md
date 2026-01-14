# nanoPCL Benchmarks

Comprehensive benchmarks for performance analysis and API design validation.

## Requirements

```bash
# Core benchmarks (no external deps)
cmake .. -DNANOPCL_BUILD_BENCHMARKS=ON

# PCL comparison benchmarks (optional)
sudo apt install libpcl-dev
```

## Building

```bash
mkdir build && cd build
cmake .. -DNANOPCL_BUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Directory Structure

```
benchmarks/
├── benchmark_common.hpp   # Shared framework (Stats, PlatformInfo, runner)
├── api_choices/           # API design decision benchmarks
├── profiling/             # Implementation optimization benchmarks
├── vs_pcl/                # PCL comparison benchmarks
└── bridge/                # ROS/PCL conversion benchmarks
```

### api_choices/

Benchmarks for API design decisions. Used to inform library API choices.

| Benchmark | Decision |
|-----------|----------|
| `copy_vs_filter_inplace.cpp` | Copy vs move filter semantics |
| `dto_vs_variadic.cpp` | PointXYZI vs variadic add() |
| `inplace_vs_copy.cpp` | Filter API design |
| `nearest_voxelhash_vs_kdtree.cpp` | Search structure tradeoffs |
| `normal_rvo_vs_outparam.cpp` | Normal estimation return type |
| `search_build_copy_vs_ptr.cpp` | Search index builder overhead |
| `search_callback_vs_vector.cpp` | Search result API |
| `search_return_vs_outparam.cpp` | Search query return type |
| `single_vs_batch_erase.cpp` | Erase API design |
| `transform_move_vs_inplace.cpp` | Transform API design |

### profiling/

Implementation optimization benchmarks for internal algorithm tuning.

| Benchmark | Purpose |
|-----------|---------|
| `benchmark_crop_angle.cpp` | atan2 vs cross-product comparison |
| `benchmark_filter.cpp` | Filter implementation strategies |
| `benchmark_icp_*.cpp` | ICP algorithm variants |
| `benchmark_pcd_io.cpp` | File I/O performance |
| `benchmark_ransac_plane.cpp` | RANSAC plane segmentation |
| `benchmark_transform.cpp` | Transform implementation (loop vs batch) |
| `benchmark_voxel_grid.cpp` | VoxelGrid copy vs move |

### vs_pcl/

Head-to-head comparisons with PCL (Point Cloud Library).

| Benchmark | Operation |
|-----------|-----------|
| `benchmark_filter.cpp` | Filter algorithms |
| `benchmark_icp.cpp` | ICP registration |
| `benchmark_normal.cpp` | Normal estimation |
| `benchmark_pointcloud.cpp` | Data structure operations |
| `benchmark_ransac_*.cpp` | RANSAC plane segmentation |
| `benchmark_sor.cpp` | Statistical outlier removal |

### bridge/

Framework integration overhead measurements.

| Benchmark | Purpose |
|-----------|---------|
| `benchmark_pcl_conversion.cpp` | PCL type conversion |
| `benchmark_ros_conversion.cpp` | ROS message conversion |

---

## Benchmark Framework

All benchmarks use `benchmark_common.hpp` for consistent methodology.

### Features

- **Unified iteration policy**: FAST(100), MEDIUM(50), SLOW(20)
- **Statistical analysis**: Mean, stddev, median, 95% CI
- **Outlier removal**: IQR method (auto-applied)
- **Platform capture**: CPU, compiler, SIMD, OpenMP
- **Consistent output**: Formatted tables with units

### Usage Example

```cpp
#include "benchmark_common.hpp"

int main() {
    benchmark::printHeader("My Benchmark");
    benchmark::PlatformInfo::capture().print();

    auto stats = benchmark::run([&]() {
        return myFunction();
    });

    benchmark::printResult("myFunction", stats);
    benchmark::printFooter("Conclusion here");
    return 0;
}
```

### Stats Structure

```cpp
struct Stats {
    double mean;        // Average time (ms)
    double stddev;      // Standard deviation
    double min, max;    // Range
    double median;      // Median value
    double ci_95_low;   // 95% CI lower bound
    double ci_95_high;  // 95% CI upper bound
    size_t n;           // Sample count
    size_t n_outliers;  // Outliers removed
};
```

---

## Key Results

### ICP Registration (vs PCL)

| Points | nanoPCL | PCL | Speedup |
|--------|---------|-----|---------|
| 10,000 | 3 ms | 39 ms | **13x** |
| 50,000 | 16 ms | 394 ms | **25x** |
| 100,000 | 54 ms | 1,296 ms | **24x** |

### Normal Estimation (vs PCL)

| Points | nanoPCL | PCL | Speedup |
|--------|---------|-----|---------|
| 100,000 | ~50 ms | ~150 ms | **3x** |

### Data Structure Operations (vs PCL)

| Operation | Speedup |
|-----------|---------|
| Construction | **350x** |
| Transform | **2.2x** |
| Random Access | **1.8x** |
| Deep Copy | **1.6x** |

---

## Optimization Notes

### Transform: Per-point Loop vs Batch (2026-01)

**Conclusion**: Per-point loop is 5x faster than Eigen::Map batch.

| Implementation | Time | Relative |
|----------------|------|----------|
| Per-point loop | 0.43ms | **1.00x** |
| Eigen::Map batch | 2.32ms | 0.19x |

**Why batch is slower:**
1. Temporary 3xN matrix allocation (6MB for 500K points)
2. Multiple memory passes vs single-pass loop
3. 3x3 × 3x1 too small for BLAS benefits

See: `profiling/benchmark_transform.cpp`

---

## Running Benchmarks

```bash
# Run individual benchmark
./profiling/benchmark_voxel_grid

# Run all profiling benchmarks
for f in profiling/benchmark_*; do ./$f; done

# Run PCL comparisons (requires PCL)
./vs_pcl/benchmark_icp
```

## Adding New Benchmarks

1. Include `benchmark_common.hpp`
2. Use `benchmark::run()` for measurement
3. Use `benchmark::printResult()` for output
4. Follow naming: `benchmark_<operation>.cpp`
5. Add entry to this README
