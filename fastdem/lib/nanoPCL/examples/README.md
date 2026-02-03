# nanoPCL Examples

Hands-on examples demonstrating nanoPCL's core features. Each example is self-contained and uses KITTI LiDAR data included in the repository.

## Quick Start

```bash
# Build all examples
cmake -B build -DBUILD_EXAMPLES=ON
cmake --build build

# Run an example
./build/examples/01_quick_start
```


## Example Overview

| # | Example | What You'll See |
|---|---------|-------------------|
| 01 | [Quick Start](#01-quick-start) | Basic pipeline: load, filter, downsample |
| 02 | [Channels](#02-channels) | SoA design, lazy channel allocation |
| 03 | [Filtering](#03-filtering) | CropBox, CropRange, VoxelGrid |
| 04 | [Registration](#04-registration) | ICP, Point-to-Plane ICP, GICP |
| 05 | [Segmentation](#05-segmentation) | Ground removal, Euclidean clustering |
| 06 | [I/O](#06-io) | Load/save PCD and KITTI BIN formats |

---

## 01 Quick Start

**Goal:** Build a typical LiDAR preprocessing pipeline.

**What you'll learn:**
- Loading KITTI point cloud data
- VoxelGrid downsampling for efficiency
- CropBox filtering to extract region of interest

```bash
./build/examples/01_quick_start
```

**Output:**
```
=== nanoPCL Quick Start ===

[Input]  Loaded 124668 points from KITTI

[Step 1] VoxelGrid (0.2m)
         Output: 25769 points (79.3% reduction)
         Time:   5.4 ms

[Step 2] CropBox (10x10x3m region)
         Output: 2692 points
         Time:   0.1 ms

[Summary]
         Total:  124668 -> 2692 points
         Time:   5.5 ms
```

**Key code:**
```cpp
auto cloud = io::loadKITTI("data/kitti/000000.bin");
auto downsampled = filters::voxelGrid(cloud, 0.2f);
auto cropped = filters::cropBox(downsampled, Point(-5,-5,-0.5), Point(5,5,2.5));
```

<!-- ![01_quick_start](assets/01_quick_start.png) -->

---

## 02 Channels

**Goal:** Understand nanoPCL's Structure-of-Arrays (SoA) design.

**What you'll learn:**
- Lazy channel allocation (pay only for what you use)
- Type-safe channel access with strong types
- Bulk channel operations for cache efficiency

```bash
./build/examples/02_channels
```

**Key concepts:**

```cpp
// Channels are created automatically when used
cloud.add(x, y, z, Intensity(0.5f), Ring(3));

// Pre-declare for performance with large data
cloud.useIntensity();
cloud.useRing();
cloud.reserve(100000);

// Bulk access (cache-friendly)
for (float& val : cloud.intensities()) {
    val *= 2.0f;
}
```

**Available channels:**
| Channel | Type | Usage |
|---------|------|-------|
| Intensity | float | `Intensity(0.5f)` |
| Ring | uint16 | `Ring(5)` |
| Time | float | `Time(0.001f)` |
| Color | RGB | `Color(255, 0, 0)` |
| Label | uint32 | `Label(42)` |

<!-- ![02_channels](assets/02_channels.png) -->

---

## 03 Filtering

**Goal:** Apply various spatial filters to point clouds.

**What you'll learn:**
- CropBox: Extract points within a 3D box
- CropRange: Filter by distance from sensor
- CropAngle: Extract angular sector (e.g., front-facing)
- CropZ: Filter by height
- VoxelGrid: Spatial downsampling

```bash
./build/examples/03_filtering
```

**Output:**
```
=== nanoPCL Filtering ===

[Input] 124668 points

[CropBox]    [-5,5]^3       62500 pts  (2.1 ms)
[CropRange]  [2m, 8m]       45230 pts  (1.8 ms)
[CropZ]      [-2m, 2m]      89450 pts  (1.5 ms)
[CropAngle]  [-45, +45 deg] 31250 pts  (1.9 ms)
[VoxelGrid]  0.5m           8750 pts   (4.2 ms)
```

**Pipeline example (move semantics for efficiency):**
```cpp
auto result = filters::voxelGrid(
    filters::cropZ(
        filters::cropRange(cloud, 1.0f, 50.0f),
        -2.0f, 3.0f),
    0.3f);
```

<!-- ![03_filtering](assets/03_filtering.png) -->

---

## 04 Registration

**Goal:** Align two point clouds using ICP variants.

**What you'll learn:**
- Point-to-Point ICP (basic)
- Point-to-Plane ICP (requires normals, faster convergence)
- GICP (requires covariances, most robust)

```bash
./build/examples/04_registration
```

**Output:**
```
=== nanoPCL Registration ===

[Target] 20397 points
[Source] 20397 points
[Ground Truth] t=(-0.53 -0.25 -0.10)

[1] Point-to-Point ICP
    Converged: no
    Iterations: 50
    Translation: (-0.55 -0.24 -0.04)

[2] Point-to-Plane ICP
    Converged: yes
    Iterations: 12
    Translation: (-0.53 -0.25 -0.10)

[3] Generalized ICP (GICP)
    Converged: yes
    Iterations: 7
    Translation: (-0.53 -0.25 -0.10)
```

**Key code:**
```cpp
// Basic ICP
auto result = registration::alignICP(source, target);

// Point-to-Plane (needs normals)
geometry::estimateNormals(target, 1.0f);
auto result = registration::alignPlaneICP(source, target);

// GICP (needs covariances)
geometry::estimateCovariances(source, 1.0f);
geometry::estimateCovariances(target, 1.0f);
auto result = registration::alignGICP(source, target);
```

<!-- ![04_registration](assets/04_registration.png) -->

---

## 05 Segmentation

**Goal:** Separate ground from obstacles and cluster objects.

**What you'll learn:**
- Ground segmentation for driving scenarios
- RANSAC plane fitting
- Euclidean clustering for object detection

```bash
./build/examples/05_segmentation
```

**Output:**
```
=== nanoPCL Segmentation ===

[Input] 124668 points

[1] Ground Segmentation
    Ground points:   89234
    Obstacle points: 35434
    Time: 12.5 ms

[2] Euclidean Clustering
    Found 15 clusters
    Cluster 0: 2340 points
    Cluster 1: 1820 points
    ...
    Time: 8.3 ms

[3] RANSAC Plane Fitting
    Plane inliers: 87650
    Plane: [0.01 0.02 0.99 -0.15]
    Time: 5.2 ms
```

**Key code:**
```cpp
// Ground segmentation
auto result = segmentation::segmentGround(cloud);
auto obstacles = cloud.extract(result.obstacles);

// Euclidean clustering
auto clusters = segmentation::euclideanCluster(obstacles, 0.5f);
for (size_t i = 0; i < clusters.numClusters(); ++i) {
    auto obj = obstacles.extract(clusters.clusterIndices(i));
}
```

<!-- ![05_segmentation](assets/05_segmentation.png) -->

---

## 06 I/O

**Goal:** Read and write point cloud files.

**What you'll learn:**
- Load/save PCD format (ASCII and binary)
- Load/save BIN format (KITTI)
- Round-trip data integrity

```bash
./build/examples/06_io
```

**Supported formats:**
| Format | Extension | Notes |
|--------|-----------|-------|
| PCD ASCII | .pcd | Human-readable, larger file |
| PCD Binary | .pcd | Compact, fast I/O |
| KITTI BIN | .bin | 4 floats per point (x,y,z,intensity) |

**Key code:**
```cpp
// Load
auto cloud = io::loadPCD("input.pcd");
auto cloud = io::loadKITTI("000000.bin");

// Save
io::savePCD("output.pcd", cloud, io::PCDFormat::BINARY);
io::saveBIN("output.bin", cloud);
```

Output files are saved to `examples/results/`.

---

## Rerun Visualization

Each example has a Rerun-enabled version in `examples/rerun/` for interactive 3D visualization.

```bash
# Build with Rerun support
cmake -B build -DBUILD_EXAMPLES=ON -DUSE_RERUN=ON
cmake --build build

# Run (opens Rerun viewer)
./build/examples/rerun/01_quick_start
```

### Remote Visualization

To view on a remote machine:
```bash
# On PC (serve mode)
RERUN_SERVE=1 ./build/examples/rerun/01_quick_start

# On remote machine
rerun --connect rerun+http://<PC_IP>:9876/proxy
```

---

## Data

Sample KITTI data is included in `data/kitti/`:

```
data/kitti/
├── 000000.bin    # LiDAR scan frame 0
├── 000001.bin    # LiDAR scan frame 1
├── 000002.bin    # LiDAR scan frame 2
├── poses.txt     # Ground truth poses
└── LICENSE.txt   # KITTI license
```

To use your own KITTI data, download from [cvlibs.net/datasets/kitti](https://www.cvlibs.net/datasets/kitti/).
