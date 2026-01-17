# Height Mapping Examples

Example programs demonstrating the height_mapping API.

## Build

```bash
cd <your_catkin_ws>
catkin build height_mapping -DBUILD_EXAMPLES=ON -DCMAKE_BUILD_TYPE=Release
```

## Examples

| # | Name | Difficulty | Description |
|---|------|------------|-------------|
| 01 | hello_mapping | Basic | Mapper fundamentals |

## Run

After building, executables are in the catkin devel space:

```bash
./devel/lib/height_mapping/01_hello_mapping
```

## Example Structure

Each example directory contains:
- `main.cpp` - Source code
- `README.md` - Documentation with learning objectives
- `CMakeLists.txt` - Build configuration

## Common Utilities

The `common/` directory provides shared utilities:

- `timer.h` - Simple performance measurement
- `data_loader.h` - Synthetic point cloud generation
- `visualization.h` - Map statistics and image export

## Roadmap

Planned examples:
- 02_config_loading - YAML configuration loading
- 03_multi_scan_fusion - Multiple scan integration
- 04_custom_provider - Mapper with Provider interfaces
- 05_height_map_io - Save/load height maps
- recipes/ - Advanced usage patterns
