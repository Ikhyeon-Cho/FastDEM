# 01. Hello Mapping

Basic Mapper usage example.

## Learning Objectives

- Create a `Mapper` with configuration
- Generate synthetic point cloud data
- Integrate point clouds into the height map
- Access height map data (statistics, cell values)
- Export height map as image

## Key Concepts

### Mapper

`Mapper` is the simplest API for height mapping. It accepts transforms directly in the `integrate()` call, making it ideal when transforms are already available.

```cpp
// Create session
height_mapping::Mapper session(config);

// Integrate point cloud
session.integrate(cloud, T_base_sensor, T_map_base);

// Access result
const auto& map = session.map();
```

### Transform Convention

- `T_base_sensor`: Static calibration (sensor → robot base)
- `T_map_base`: Robot pose (robot base → map frame)

For a sensor mounted at the robot origin with no movement, both transforms are identity.

## Build

```bash
cd <your_catkin_ws>
catkin build height_mapping -DBUILD_EXAMPLES=ON
```

## Run

```bash
./devel/lib/height_mapping/01_hello_mapping
```

## Expected Output

```
=== 01_hello_mapping ===
Basic Mapper usage example

Configuration:
  Map size:    10 x 10 m
  Resolution:  0.1 m/cell
  Estimator:   incremental_mean

Created Mapper
Generating synthetic terrain...
Generated 50000 points
[Integration] XX.XX ms

=== HeightMap Statistics ===
Grid size:   100 x 100 cells
Resolution:  0.1 m/cell
Physical:    10 x 10 m
Coverage:    ~50% (depends on random data)
...

Saved: hello_mapping_output.ppm
Done!
```

## Output Files

- `hello_mapping_output.ppm`: Colorized height map image (Viridis colormap)
