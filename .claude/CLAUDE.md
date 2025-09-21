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
```

### Running the System
```bash
# Run height mapping with default configuration
roslaunch height_mapping height_mapping.launch

# Run with visualization (includes RViz)
roslaunch height_mapping height_mapping_with_rviz.launch

# Run with custom config files
roslaunch height_mapping height_mapping.launch ros_config:=path/to/ros_params.yaml engine_config:=path/to/engine_config.yaml

# Available pre-configured profiles
roslaunch height_mapping height_mapping.launch engine_config:=$(rospack find height_mapping)/config/engine_config_fast.yaml     # Fast processing mode
roslaunch height_mapping height_mapping.launch engine_config:=$(rospack find height_mapping)/config/engine_config_accurate.yaml # Accurate mapping mode
```

## Architecture Overview

### Package Structure
The codebase consists of four main packages with clear separation of concerns:

1. **pipeline_core**: Generic pipeline processing framework
   - Reusable pipeline architecture (pipeline_core/include/pipeline_core/)
   - Stage registry and factory pattern for extensibility
   - Configuration loading and validation system

2. **height_map_core**: Core data structures for height map representation
   - Grid map data structure (height_map_core/include/height_map_core/data/)
   - Height map operations and utilities
   - Point cloud data structures

3. **height_mapping_core**: Processing pipeline and algorithms
   - Pipeline stages implementation (height_mapping_core/include/height_mapping_core/stages/)
   - Height estimation algorithms
   - Mapping engine coordinating the pipeline

4. **height_mapping_ros**: ROS integration layer
   - ROS node implementation (height_mapping_ros/src/height_mapping_node.cpp)
   - Publishers/subscribers for ROS topics
   - Configuration bridge between ROS and core libraries

### Pipeline Architecture

The system uses a modular pipeline where stages are executed sequentially on a shared MappingContext:

#### Core Components
- **MappingContext** (height_mapping_core/include/height_mapping_core/pipeline/mapping_context.h): Shared data container
- **MappingEngine** (height_mapping_core/include/height_mapping_core/mapping_engine.h): Pipeline executor
- **Stage** (pipeline_core/include/pipeline_core/stage.h): Base class for all processing stages

#### Available Pipeline Stages
Configured through engine_config.yaml, executed in order:

1. **TransformStage**: Transforms point clouds to target frame
2. **VoxelFilterStage**: Downsamples using voxel grid (configurable resolution)
3. **PassthroughFilterStage**: Filters by distance/height thresholds
4. **FastHeightFilterStage**: Quick outlier removal
5. **RaycastingStage**: Projects 3D points to 2.5D grid
6. **HeightEstimationStage**: Estimates height using selected algorithm
7. **MultiSensorSyncStage**: Synchronizes multi-sensor data
8. **MapOriginUpdateStage**: Updates map position with robot movement
9. **GlobalMappingStage**: Aggregates local maps into global map

### Configuration System

#### Configuration Files Location
- **ROS parameters**: height_mapping_ros/config/ros_params.yaml
- **Engine configs**: height_mapping_ros/config/engine_config*.yaml
- **Node-specific**: height_mapping_ros/config/*_node.yaml

#### Configuration Hierarchy
1. Launch file specifies config paths
2. ros_params.yaml loads and references engine_config.yaml path
3. Engine config defines pipeline stages and parameters
4. Each stage reads its parameters from the config

### Height Estimation Algorithms

Configured via `estimator_type` in HeightEstimationStage:

- **INCREMENTAL_MEAN**: Statistical mean with variance tracking
- **KALMAN**: Recursive Bayesian estimation with uncertainty
- **MOVING_AVERAGE**: Sliding window average

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
2. Inherit from pipeline::Stage
3. Implement process() and configure() methods
4. Register stage in stage factory
5. Add configuration to engine_config.yaml

### Modifying Height Estimation
Edit height_mapping_core/include/height_mapping_core/stages/height_estimation.h
to add new estimation methods or modify existing algorithms.

### Debugging Pipeline Execution
- Enable debug output in ros_params.yaml
- Use RViz visualization launch file
- Check stage execution order in engine_config.yaml
- Monitor /rosout for stage processing messages