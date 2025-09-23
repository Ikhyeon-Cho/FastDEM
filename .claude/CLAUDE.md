# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### Building the Package
```bash
# Build all height_mapping packages (core, ROS, and pipeline_core)
catkin build height_mapping

# Build with release optimizations (strongly recommended for performance)
catkin build height_mapping -DCMAKE_BUILD_TYPE=Release

# Clean build
catkin clean height_mapping
catkin build height_mapping

# Build specific package only
catkin build height_mapping_core  # Core library only
catkin build height_mapping_ros   # ROS node only
catkin build pipeline_core        # Pipeline framework only
catkin build height_map_core      # Core data structures only

# Build and run tests
catkin build height_mapping --catkin-make-args run_tests
```

### Running the System
```bash
# Run height mapping with default configuration
roslaunch height_mapping height_mapping.launch

# Run with visualization (includes RViz)
roslaunch height_mapping height_mapping_with_rviz.launch

# Run with custom config files
roslaunch height_mapping height_mapping.launch ros_config:=path/to/ros_params.yaml engine_config:=path/to/engine_config.yaml

# Run with pipeline configuration
roslaunch height_mapping height_mapping.launch pipeline_config:=$(rospack find height_mapping)/config/pipeline_config.yaml
```

### Testing and Validation
```bash
# Check if nodes are running
rosnode list | grep height_mapping

# Monitor point cloud input rate
rostopic hz /velodyne/tilted/velodyne_points

# Monitor height map output
rostopic echo /height_map/info

# Visualize in RViz
rosrun rviz rviz -d $(rospack find height_mapping)/rviz/height_mapping.rviz
```

## Architecture Overview

### Package Structure
The codebase consists of three main packages with clear separation of concerns:

1. **pipeline_core**: Generic pipeline processing framework (ROS-agnostic)
   - Reusable pipeline architecture (pipeline_core/include/pipeline_core/)
   - Stage registry and factory pattern for extensibility
   - Configuration loading and validation system

2. **height_mapping_core**: Processing pipeline and algorithms (depends on grid_map_core)
   - Pipeline stages implementation (height_mapping_core/include/height_mapping_core/stages/)
   - Height estimation algorithms (height_mapping_core/include/height_mapping_core/estimators/)
   - Mapping engine coordinating the pipeline
   - Grid map data structures via grid_map_core dependency

3. **height_mapping_ros**: ROS integration layer
   - ROS node implementation (height_mapping_ros/src/height_mapping_node.cpp)
   - Publishers/subscribers for ROS topics
   - Configuration bridge between ROS and core libraries
   - TF2-based transform providers

### Pipeline Architecture

The system uses a modular pipeline where stages are executed sequentially on a shared MappingContext:

#### Core Components
- **MappingContext** (height_mapping_core/include/height_mapping_core/pipeline/mapping_context.h): Shared data container
- **MappingEngine** (height_mapping_core/include/height_mapping_core/mapping_engine.h): Pipeline executor
- **Stage** (pipeline_core/include/pipeline_core/stage.h): Base class for all processing stages

#### Available Pipeline Stages
Configured through engine_config.yaml or pipeline_config.yaml, executed in order:

1. **PointCloudTransform**: Transforms point clouds between coordinate frames (base_link/map)
2. **VoxelFilter**: Downsamples using voxel grid (configurable resolution and method)
3. **PassthroughFilter**: Filters by spatial bounds and distance thresholds
4. **Raycasting**: Projects 3D points to 2.5D grid with ground correction
5. **HeightEstimation**: Estimates height using selected algorithm
6. **MultiSensorSync**: Synchronizes multi-sensor data streams
7. **MapOriginUpdate**: Updates map position following robot movement
8. **GlobalMapping**: Aggregates local maps into extended global map

### Configuration System

#### Configuration Files Location
- **ROS parameters**: height_mapping_ros/config/ros_params.yaml
- **Engine configs**: height_mapping_ros/config/engine_config.yaml
- **Pipeline config**: height_mapping_ros/config/pipeline_config.yaml

#### Configuration Hierarchy
1. Launch file specifies config paths
2. ros_params.yaml loads and references engine_config.yaml path
3. Engine/Pipeline config defines stages and parameters
4. Each stage reads its parameters from the config

### Height Estimation Algorithms

Located in height_mapping_core/include/height_mapping_core/estimators/:

- **incremental_mean**: Statistical mean with variance tracking
- **kalman_filter**: Recursive Bayesian estimation with configurable noise parameters
- **moving_average**: Sliding window average with configurable window size

Factory pattern (EstimatorFactory) creates estimators based on configuration.

## Key Implementation Patterns

### Memory Management
- Processing context pool for efficient memory reuse
- Pre-allocated grid map layers
- Smart pointers for automatic resource management

### Thread Safety
- ROS spinner handles callbacks
- Pipeline executes synchronously per callback
- Shared resources protected by the context

### Performance Considerations
- Release build provides ~3-5x performance improvement
- Voxel filtering resolution impacts processing speed
- Map resolution vs accuracy tradeoff
- Processing rate can be configured per node

### Error Handling
- Pipeline stages validate input/output
- Configuration validation at startup
- Transform lookup with timeout handling

## Common Development Tasks

### Adding a New Pipeline Stage
1. Create header in height_mapping_core/include/height_mapping_core/stages/
2. Create implementation in height_mapping_core/src/stages/
3. Inherit from pipeline::Stage
4. Implement processImpl() and configure() methods
5. Add REGISTER_STAGE(YourStageName) at the end of your .cpp file
6. Add stage configuration to unified_config.yaml (no code changes needed in MappingEngine)

Example:
```cpp
// my_stage.h
class MyStage : public pipeline::Stage {
  ProcessResult processImpl(pipeline::Context& ctx) override;
  void configure(const std::map<std::string, std::string>& params) override;
};

// my_stage.cpp
#include "height_mapping_core/stages/my_stage.h"
// ... implementation ...
REGISTER_STAGE(MyStage)  // Auto-registers with factory
```

### Adding a New Height Estimator
1. Create header in height_mapping_core/include/height_mapping_core/estimators/
2. Inherit from HeightEstimatorBase
3. Implement estimate() and reset() methods
4. Register in EstimatorFactory
5. Update HeightEstimationStage to use new estimator

### Modifying ROS Adapters
- Transform provider: height_mapping_ros/include/height_mapping_ros/adapters/tf2_transform.h
- Point cloud converter: height_mapping_ros/include/height_mapping_ros/adapters/pointcloud_converter.h
- Located in height_mapping_ros/src/adapters/

### Debugging Pipeline Execution
- Enable debug output in ros_params.yaml
- Use RViz visualization launch file
- Check stage execution order in pipeline_config.yaml
- Monitor /rosout for stage processing messages
- Use rqt_graph to visualize node connections

### Working with Configurations
```bash
# Validate configuration before running
rosparam load height_mapping_ros/config/ros_params.yaml
rosparam get /height_mapping

# Check active pipeline stages
grep -A 20 "stages:" height_mapping_ros/config/pipeline_config.yaml

# Monitor runtime parameters
rosparam get /height_mapping/height_estimation/method
```

## Configuration System

### Architecture
The system uses a unified YAML configuration that controls both engine settings and pipeline composition:

1. **MappingEngine Constructors**:
   - `MappingEngine(config_file, tf_provider)`: Loads everything from YAML file
   - `MappingEngine(EngineConfig, tf_provider)`: Manual configuration for testing

2. **Configuration Loading**:
   - Engine settings (map size, resolution) loaded from `engine` section
   - Pipeline automatically built from `pipeline` section using `ConfigLoader::createPipelineFromFile()`
   - Falls back to hardcoded default pipeline if YAML loading fails

3. **Stage Registration**:
   - All stages auto-register using `REGISTER_STAGE(StageName)` macro
   - `StageRegistry::create()` dynamically instantiates stages by name
   - No manual factory updates needed when adding new stages

### Configuration Files
- **unified_config.yaml**: Complete configuration with all stages
- **presets/fast.yaml**: Optimized for speed (lower resolution, simplified pipeline)
- **presets/accurate.yaml**: Optimized for quality (higher resolution, full pipeline)

### Configuration Structure
```yaml
# unified_config.yaml structure
engine:
  map:
    width: 10.0        # Map dimensions in meters
    height: 10.0
    resolution: 0.1    # Meters per cell
    frame_id: "map"
    robot_centric: true
  runtime:
    thread_safe: true
    auto_reset_on_error: false

pipeline:
  stop_on_error: true
  stages:
    - name: "VoxelFilter"     # Stage name must match registered class
      enabled: true
      params:
        voxel_size: 0.05
        reduction_method: "centroid"
    - name: "HeightEstimation"
      enabled: true
      params:
        estimator_type: "kalman_filter"
```

### Usage
```bash
# Default unified configuration
roslaunch height_mapping height_mapping.launch

# Custom configuration file
roslaunch height_mapping height_mapping.launch config:=/path/to/custom.yaml

# Use preset configurations
roslaunch height_mapping height_mapping_fast.launch      # Fast preset
roslaunch height_mapping height_mapping_accurate.launch  # Accurate preset

# Override specific parameters
roslaunch height_mapping height_mapping.launch \
  config:=$(rospack find height_mapping)/config/unified_config.yaml \
  input_topic:=/velodyne_points \
  publish_rate:=20.0
```