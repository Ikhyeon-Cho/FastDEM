# Height Mapping Refactoring Plan

## Status: 70% Complete

## Goal
Separate pipeline infrastructure from domain logic by splitting into `height_mapping_core` (pure algorithms) and `height_mapping_pipeline` (pipeline adapters).

## Architecture Overview

```
┌─────────────────────────┐
│   height_mapping_ros    │  Layer 4: ROS Integration
├─────────────────────────┤
│ height_mapping_pipeline │  Layer 3: Pipeline Adaptation
├─────────────────────────┤
│  height_mapping_core    │  Layer 2: Domain Logic (Pure)
├─────────────────────────┤
│    pipeline_core        │  Layer 1: Generic Pipeline Infrastructure
└─────────────────────────┘
```

## Phase 1: Package Structure Setup

- [x] **Create height_mapping_pipeline package**
  - [x] Create package directory: `height_mapping_pipeline/`
  - [x] Create include directory: `height_mapping_pipeline/include/height_mapping_pipeline/`
  - [x] Create src directory: `height_mapping_pipeline/src/`
  - [x] Create CMakeLists.txt with dependencies on `pipeline_core` and `height_mapping_core`
  - [x] Create package.xml with proper dependencies

## Phase 2: Core Algorithm Refactoring (Stateful Design)

### Ground Segmentation
- [x] **Extract algorithm from current stage**
  - [x] Create `height_mapping_core/include/height_mapping_core/algorithms/grid_ground_segmentation.h`
  - [x] Define `GridGroundSegmentation::Config` struct with parameters
  - [x] Implement stateful `GridGroundSegmentation` class with Config in constructor
  - [x] Move algorithm logic from stage to new class
  - [x] Add `segmentInPlace()` method that processes points

- [x] **Create pipeline adapter**
  - [x] Create `GridGroundSegmentationStage` in `height_mapping_pipeline/stages/`
  - [x] Refactor to use new `algorithms::GridGroundSegmentation` class
  - [x] Implement `configure()` to parse YAML params into Config
  - [x] Update `processImpl()` to call `segmenter_->segmentInPlace()`

### Statistical Ground Segmentation
- [x] **Extract algorithm**
  - [x] Create `height_mapping_core/include/height_mapping_core/algorithms/statistical_ground_segmentation.h`
  - [x] Define `StatisticalGroundSegmentation::Config` struct
  - [x] Implement stateful class with algorithm logic
  - [x] Move algorithm logic to core

- [x] **Create pipeline adapter**
  - [x] Create `height_mapping_pipeline/stages/statistical_ground_segmentation.h`
  - [x] Implement Stage adapter using core algorithm

### Height Estimation
- [x] **Refactor existing estimators to stateful**
  - [x] Estimators already stateful with factory pattern
  - [x] `HeightEstimator` class with Config
  - [x] Factory creates estimators based on type
  - [x] Config passed via factory

- [x] **Update HeightEstimation stage**
  - [x] Created in `height_mapping_pipeline/stages/height_estimation.h`
  - [x] Uses stateful estimator pattern via `algorithms::HeightEstimator`
  - [x] Parse YAML config in `configure()` method

### Other Stages
- [x] **VoxelFilter**
  - [x] Create `algorithms::VoxelFilter` class with Config
  - [x] Create pipeline adapter stage

- [x] **PassthroughFilter**
  - [x] Create `algorithms::PassthroughFilter` class
  - [x] Create pipeline adapter stage

- [x] **TransformCloud**
  - [x] Create `algorithms::CloudTransformer` class
  - [x] Create pipeline adapter stage

- [x] **Raycasting**
  - [x] Create `algorithms::Raycaster` class with Config
  - [x] Create pipeline adapter stage

- [x] **MoveOrigin**
  - [x] Created as `algorithms::MapOriginUpdater` in core
  - [x] Pipeline adapter in `move_origin.h`

## Phase 3: Context Restructuring

- [x] **Move MappingContext to pipeline layer**
  - [x] Moved from `height_mapping_core/pipeline/mapping_context.h`
  - [x] To `height_mapping_pipeline/include/height_mapping_pipeline/mapping_context.h`
  - [x] Remove pipeline dependencies from core

- [x] **MappingEngine moved to pipeline layer**
  - [x] Moved to `height_mapping_pipeline/mapping_engine.h`
  - [x] Core package now has zero pipeline dependencies

- [ ] **Add branch support to MappingContext**
  - [ ] Add `ground_cloud_` member
  - [ ] Add `non_ground_cloud_` member
  - [ ] Add accessor methods

## Phase 4: Parallel Processing Support

- [ ] **Implement BranchStage in pipeline_core**
  - [ ] Create `pipeline_core/include/pipeline_core/branch_stage.h`
  - [ ] Implement base class with `branches_` map
  - [ ] Add parallel execution in `processImpl()`
  - [ ] Support YAML configuration of branches

- [ ] **Create ParallelProcessingStage**
  - [ ] Create in `height_mapping_pipeline/stages/`
  - [ ] Configure ground and obstacle processing branches
  - [ ] Test with sample pipelines

## Phase 5: Build System Updates

- [x] **Update height_mapping_core CMakeLists.txt**
  - [x] Remove pipeline_core dependency
  - [x] Add algorithm source files
  - [x] Keep only domain dependencies (Eigen, grid_map)

- [x] **Create height_mapping_pipeline CMakeLists.txt**
  - [x] Add dependencies: pipeline_core, height_mapping_core
  - [x] Stage adapters are header-only
  - [x] Install headers and libraries

- [x] **Update height_mapping_ros CMakeLists.txt**
  - [x] Add height_mapping_pipeline dependency
  - [x] Update include paths

## Phase 6: Configuration Updates

- [ ] **Update YAML configuration loading**
  - [ ] Move YAML parsing to pipeline layer
  - [ ] Create ConfigLoader in pipeline package
  - [ ] Update stage configurations for new structure

- [ ] **Create algorithm presets**
  - [ ] Fast configuration presets
  - [ ] Accurate configuration presets
  - [ ] Custom configuration examples

## Phase 7: Testing

- [ ] **Core algorithm unit tests**
  - [ ] Test algorithms without pipeline dependency
  - [ ] Test Config validation
  - [ ] Test stateful behavior

- [ ] **Pipeline integration tests**
  - [ ] Test YAML to Config conversion
  - [ ] Test stage adapters
  - [ ] Test parallel processing

- [ ] **System tests**
  - [ ] Test complete pipeline execution
  - [ ] Test ROS node integration
  - [ ] Performance benchmarks

## Phase 8: Documentation

- [ ] **Update README files**
  - [ ] Document new package structure
  - [ ] Explain separation of concerns
  - [ ] Usage examples

- [ ] **API documentation**
  - [ ] Document core algorithms API
  - [ ] Document pipeline adapters
  - [ ] Configuration guide

## Migration Notes

### Breaking Changes
1. `MappingContext` moved to `height_mapping_pipeline` namespace
2. All stages moved to `height_mapping_pipeline/stages/`
3. Algorithm configs now passed in constructor, not methods

### Backward Compatibility
- Keep old stage files temporarily with deprecation warnings
- Provide migration guide for existing configs

## Success Criteria

- [x] Core package has zero pipeline dependencies
- [x] All algorithms work in stateful mode
- [ ] Parallel processing works with shared context
- [ ] All tests pass
- [ ] Performance is maintained or improved
- [x] Clear separation between layers

## Completed vs Remaining

### ✅ Completed (Phase 1, 2, 3 partial, 5)
- Package structure setup
- All core algorithms extracted and stateful
- MappingContext and MappingEngine moved to pipeline layer
- Build system fully updated
- Core package has zero pipeline dependencies

### ⏳ Remaining (Phase 3 partial, 4, 6, 7, 8)
- Branch support in MappingContext for ground/non-ground
- Parallel processing support (BranchStage)
- Configuration updates and presets
- Testing (unit, integration, system)
- Documentation

## Timeline Estimate

- Completed: ~3 days of work done
- Remaining: ~2 days needed