# FastDEM Examples

Example programs demonstrating the FastDEM API.

## Build

```bash
cd FastDEM/fastdem
mkdir -p build && cd build
cmake .. -DFASTDEM_BUILD_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Examples

| # | Name | Difficulty | Description |
|---|------|------------|-------------|
| 01 | hello_mapping | Basic | Map setup, offline integration, elevation access |
| 02 | config_loading | Basic | YAML configuration loading (LOCAL / GLOBAL presets) |
| 03 | estimator_comparison | Intermediate | Compare Kalman, Welford, and P2 Quantile estimators |
| 04 | transform_provider | Intermediate | Online mode with Calibration / Odometry interfaces |

## Run

After building, executables are in the build directory:

```bash
./build/examples/01_hello_mapping/01_hello_mapping
./build/examples/02_config_loading/02_config_loading
./build/examples/03_estimator_comparison/03_estimator_comparison
./build/examples/04_transform_provider/04_transform_provider
```

## Common Utilities

The `common/` directory provides shared utilities:

- `timer.hpp` - Simple performance measurement
- `data_loader.hpp` - Synthetic point cloud generation
- `visualization.hpp` - Map statistics and image export
