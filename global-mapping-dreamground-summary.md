# Global Mapping & DreamGround Height Estimation Summary

## Global Mapping Implementation

### Current System Analysis
- **Local mapping**: 15m×15m robot-centric maps with dynamic origin (MoveOrigin stage)
- **Global mapping ready**: Set `robot_centric: false` or `update_mode: "fixed"` in config
- **Memory considerations**: 200m×200m @ 0.1m = 4M cells (16MB per layer)

### Publishing Bottleneck Solutions

#### Multi-Tier Publishing (Recommended)
```yaml
topics:
  /map/local:    30×30m @ 10Hz   # Active mapping area
  /map/recent:   60×60m @ 2Hz    # Recent exploration
  /map/global:   200×200m @ 0.2Hz # Full archive
  /map/updates:  Changes @ 10Hz   # Incremental updates
```

#### PointCloud2 Optimization
- **Sparse publishing**: Only valid cells (70-90% reduction)
- **Distance-based LOD**: Full res <20m, medium <50m, low >50m
- **Height filtering**: Only publish interesting heights

## Height Estimation for Multi-Hit Cells

### Problem: Multiple LiDAR channels hit same grid cell
16-channel LiDAR creates vertical point columns requiring height selection strategy.

### Recommended: Lowest Stable Cluster
```cpp
float selectHeight(std::vector<float>& z_values) {
    std::sort(z_values.begin(), z_values.end());
    auto clusters = clusterByGap(z_values, 0.3); // 30cm threshold
    return computeRobustMean(clusters[0]);       // Ground cluster
}
```

**Why optimal**:
- Represents traversable ground surface
- Robust to noise (uses cluster mean)
- Naturally filters dynamic objects

## Scan-Elevation Map Dataset Construction

### Core Challenges

1. **Dynamic Objects**: People/cars in scan but not in elevation map
2. **Coordinate System**: Need robot-centric (0,0) at LiDAR origin
3. **Sensor Height**: Handle base_link transformations properly

### Dataset Generation Pipeline

#### Step 1: Build High-Quality Global Map
- Process entire sequence with Kalman filter
- Generate temporally-fused static environment map
- This becomes ground truth reference

#### Step 2: Generate Training Pairs
For each timestamp T:
- **Input (X)**: Raw LiDAR scan with dynamic objects
- **Target (Y)**: Robot-centric crop from global map

### Dynamic Object Handling

#### Multi-Layer Approach (Recommended)
```yaml
outputs:
  ground_map:     Lowest stable cluster height
  obstacle_mask:  Binary mask of dynamic objects
  uncertainty:    Cluster separation metric
```

Model learns to:
- Detect dynamic objects
- Inpaint ground beneath them
- Quantify prediction uncertainty

## DreamGround Context Integration

### Preprocessing Strategy for DreamGround

```cpp
struct DreamGroundCell {
    float elevation;      // Lowest cluster (ground)
    float clearance;      // Vertical space
    float uncertainty;    // Epistemic uncertainty
    bool is_occluded;    // Dynamic object flag
};
```

### Why Lowest Stable Cluster for DreamGround

1. **Ground Navigation Focus**: Provides traversable surface
2. **Uncertainty Quantification**: Natural from cluster analysis
3. **Dynamic Filtering**: Automatic via height clustering
4. **Curb/Ramp Detection**: Preserves critical low features

### Implementation Pipeline

```yaml
GridRasterization:
  grid_size: 0.1m
  height_policy: multi_layer
  outputs:
    - ground_height    # Primary navigation target
    - obstacle_height  # Collision avoidance
    - uncertainty      # Risk assessment
```

## Key Decisions

1. **Publishing**: Use multi-tier + PointCloud2 for scalability
2. **Height Selection**: Lowest stable cluster for ground truth
3. **Dynamic Objects**: Multi-layer with uncertainty masks
4. **Preprocessing**: Grid-aligned rasterization matching map resolution
5. **DreamGround**: Focus on ground reconstruction with calibrated uncertainty

## Critical Implementation Notes

- Temporal fusion (Kalman) essential for ground truth quality
- Robot-centric transformation via inverse mapping (avoid direct rotation)
- Cluster gap threshold (30cm) critical for ground/obstacle separation
- Uncertainty quantification enables risk-aware navigation
- Multi-layer output supports multi-task learning