# Incremental Implementation Plan

## Phase 1: Core Foundation (Essential - Do First)
**Goal:** Get basic height mapping working end-to-end

1. **Implement basic StatMean estimator**
   - Add incremental mean calculation to HeightEstimation stage
   - Simple height = (old_height * (n-1) + new_height) / n
   - Track measurement count per cell

2. **Add essential map layers**
   - elevation_min, elevation_max layers to height_map.h
   - n_measurements layer for counting points per cell
   - Basic getters/setters for these layers

3. **Fix transform handling**
   - Let each stage specify required transforms
   - PassthroughFilter: sensor→base transform
   - MapOriginUpdate: base→map transform
   - Remove fixed target_frame from TransformStage

## Phase 2: Statistical Robustness (Important)
**Goal:** Add proper variance tracking and confidence

4. **Implement Welford's algorithm for variance**
   - Proper online variance calculation in StatMean
   - Track M2 (sum of squared differences)
   - Convert to sample variance

5. **Add confidence layers**
   - standard_error = sqrt(variance/n)
   - confidence_interval = 1.96 * standard_error (95% CI)
   - Store in optional layers

## Phase 3: Advanced Estimators (Nice to have)
**Goal:** Support multiple estimation algorithms

6. **Implement Kalman filter estimator**
   - Process/measurement noise parameters
   - Kalman gain calculation
   - Adaptive variance based on distance/angle

7. **Implement Moving Average estimator**
   - Exponential weighted average
   - Alpha parameter for weight control
   - Optional adaptive weighting

## Phase 4: Sensor Features (Enhancement)
**Goal:** Support richer sensor data

8. **Add point cloud type support**
   - Template HeightEstimation for PointXYZI, PointXYZRGB
   - Add intensity layer for LiDAR
   - Add RGB layers for cameras

9. **Implement raycasting improvements**
   - Clear cells between sensor and measurement
   - Track scan heights
   - Handle occlusions properly

## Phase 5: Configuration & Polish (Final)
**Goal:** Runtime flexibility and completeness

10. **Dynamic algorithm selection**
    - Load estimator type from config
    - Parameter classes for each estimator
    - Validation and defaults

11. **Helper utilities**
    - Point cloud validation functions
    - Statistics computation helpers
    - Debug/visualization layers

## Priority Order:
1. **Must Have:** Phase 1 (Core) + Phase 2 (Statistics)
2. **Should Have:** Phase 3 (Advanced Estimators)
3. **Nice to Have:** Phase 4 (Sensor Features) + Phase 5 (Polish)

Each phase builds on the previous one and can be tested independently. Start with Phase 1 to get a working system, then incrementally add sophistication.