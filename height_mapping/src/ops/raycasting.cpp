/*
 * raycasting.cpp
 *
 * Raycasting for ghost obstacle removal using maximum ray height.
 *
 * Algorithm:
 * 1. Filter: use all downward rays (point.z < sensor.z) for maximum coverage
 * 2. Extract targets (min height per cell) and build hit_mask
 * 3. Compute raycasting_upper_bound layer (max ray height at each cell)
 * 4. Conflict detection: elevation > raycasting_upper_bound + threshold
 * 5. Temporal voting: increment conflict_count, clear when >= vote_threshold
 * 6. Reset conflict_count at target cells (measurement = not ghost)
 *
 * Key insight: Using maximum ray height ensures conservative ghost detection.
 * "All rays passed below the recorded elevation" = confirmed ghost.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/ops/raycasting.h"

#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>
#include <vector>

namespace height_mapping::ops {

namespace {

struct RayTarget {
  grid_map::Index index;
  Eigen::Vector3f point;  // Store exact point for accurate ray geometry
};

struct TargetExtraction {
  std::vector<RayTarget> targets;
  std::vector<bool> hit_mask;  // true = cell was hit in this scan
  int cols;                    // For linear index calculation
};

/**
 * @brief Extract ray targets from point cloud.
 *
 * For each cell, keeps the minimum height point as the target.
 * Also builds a hit_mask to mark which cells were measured.
 *
 * @param map Height map for coordinate conversion
 * @param cloud Point cloud (already filtered for downward rays)
 * @return Targets and hit mask
 */
TargetExtraction extractTargets(const HeightMap& map, const PointCloud& cloud) {
  const auto& size = map.getSize();
  const int rows = size(0);
  const int cols = size(1);

  // Track minimum height point per cell
  std::vector<float> min_heights(rows * cols,
                                 std::numeric_limits<float>::quiet_NaN());
  std::vector<Eigen::Vector3f> min_points(rows * cols);
  std::vector<bool> hit_mask(rows * cols, false);
  std::vector<grid_map::Index> active_indices;
  active_indices.reserve(cloud.size());

  // Use direct vector reference for cache-optimal iteration
  const auto& points = cloud.xyz();
  for (const auto& point : points) {
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      continue;
    }

    const int linear_idx = idx(0) * cols + idx(1);
    float& min_h = min_heights[linear_idx];

    if (std::isnan(min_h)) {
      min_h = point.z();
      min_points[linear_idx] = point;
      active_indices.push_back(idx);
      hit_mask[linear_idx] = true;
    } else if (point.z() < min_h) {
      min_h = point.z();
      min_points[linear_idx] = point;
    }
  }

  // Build targets from active indices
  std::vector<RayTarget> targets;
  targets.reserve(active_indices.size());
  for (const auto& idx : active_indices) {
    const int linear_idx = idx(0) * cols + idx(1);
    targets.push_back({idx, min_points[linear_idx]});
  }

  return {std::move(targets), std::move(hit_mask), cols};
}

/**
 * @brief Trace ray by sampling, tracking maximum height at each cell.
 *
 * Samples points along the ray at intervals smaller than grid resolution,
 * ensuring all cells the ray passes through are visited (supercover effect).
 * Tracks maximum ray height at each cell for conservative ghost detection.
 *
 * Excludes target cell and cells within endpoint_margin distance.
 *
 * @param map Height map (for coordinate conversion)
 * @param start 3D ray start position (sensor)
 * @param end 3D ray end position (target)
 * @param ray_max_mat Matrix tracking maximum ray height per cell
 * @param endpoint_margin Number of cells to skip before target
 */
void traceRayForMaxHeight(const HeightMap& map,
                          const Eigen::Vector3f& start,
                          const Eigen::Vector3f& end,
                          grid_map::Matrix& ray_max_mat,
                          int endpoint_margin) {
  // Ray vector and 2D length
  const Eigen::Vector3f ray = end - start;
  const float ray_length_2d = std::sqrt(ray.x() * ray.x() + ray.y() * ray.y());

  if (ray_length_2d < 1e-4f) return;

  // Normalized direction (by 2D length for consistent stepping)
  const Eigen::Vector3f dir = ray / ray_length_2d;

  // Sampling step: half of resolution for guaranteed cell coverage
  const float resolution = map.getResolution();
  const float step = resolution * 0.5f;

  // Trace range: from sensor to (ray_length - margin)
  const float margin_dist = endpoint_margin * resolution;
  const float trace_end = ray_length_2d - margin_dist;

  if (trace_end <= 0.0f) return;

  // Sample along ray, tracking maximum height
  for (float t = 0.0f; t < trace_end; t += step) {
    // 3D sample point on ray
    const Eigen::Vector3f sample = start + dir * t;

    // Get grid cell for this sample
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(sample.x(), sample.y()), idx)) {
      continue;
    }

    // Track maximum ray height (conservative for ghost detection)
    float& current_max = ray_max_mat(idx(0), idx(1));
    if (std::isnan(current_max) || sample.z() > current_max) {
      current_max = sample.z();
    }
  }
}

/**
 * @brief Compute raycasting upper bound for all targets.
 *
 * Traces rays from sensor to each target using sampling-based approach,
 * computing maximum ray height at each cell for conservative ghost detection.
 *
 * @param map Height map (for raycasting_upper_bound layer access)
 * @param sensor_origin Sensor position in map frame
 * @param sensor_idx Sensor cell index
 * @param targets Ray targets
 * @param config Raycasting configuration
 */
void computeUpperBounds(HeightMap& map,
                        const Eigen::Vector3f& sensor_origin,
                        const grid_map::Index& sensor_idx,
                        const std::vector<RayTarget>& targets,
                        const config::Raycasting& config) {
  // Get reference to upper bound layer (already initialized with NAN)
  auto& upper_bound_mat = map.get(layer::raycasting_upper_bound);

  // Trace all rays, tracking maximum height
  for (const auto& target : targets) {
    // Skip if target is same cell as sensor
    if (target.index.x() == sensor_idx.x() &&
        target.index.y() == sensor_idx.y()) {
      continue;
    }

    // Trace ray, updating maximum heights
    traceRayForMaxHeight(map, sensor_origin, target.point,
                         upper_bound_mat, config.endpoint_margin);
  }
}

/**
 * @brief Process conflicts based on upper_bound vs elevation comparison.
 *
 * Conflict condition: elevation > raycasting_upper_bound + threshold
 * (all rays passed below the recorded elevation = ghost obstacle)
 *
 * @param map Height map
 * @param hit_mask Cells hit in current scan (protected)
 * @param config Raycasting configuration
 * @param cols Grid columns
 * @return Number of cells cleared
 */
size_t processConflicts(HeightMap& map,
                        const std::vector<bool>& hit_mask,
                        const config::Raycasting& config,
                        int cols) {
  const auto& upper_bound = map.get(layer::raycasting_upper_bound);
  const auto& elevation = map.get(layer::elevation);
  auto& conflict_count = map.get(layer::conflict_count);

  const int rows = map.getSize()(0);
  size_t cells_cleared = 0;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const int linear_idx = i * cols + j;

      // Skip if hit (protected) or no ray passed
      if (hit_mask[linear_idx] || std::isnan(upper_bound(i, j))) {
        continue;
      }

      // Skip if no elevation data
      if (std::isnan(elevation(i, j))) {
        continue;
      }

      // Conflict: elevation > upper_bound + threshold
      // (all rays passed below the recorded elevation = ghost)
      if (elevation(i, j) > upper_bound(i, j) + config.height_threshold) {
        // Initialize conflict count if NaN
        if (std::isnan(conflict_count(i, j))) {
          conflict_count(i, j) = 0.0f;
        }

        conflict_count(i, j) += 1.0f;

        // Clear cell if threshold exceeded
        if (static_cast<int>(conflict_count(i, j)) >= config.vote_threshold) {
          map.clearAt(grid_map::Index(i, j));
          cells_cleared++;
        }
      }
    }
  }

  return cells_cleared;
}

}  // namespace

// =============================================================================
// Main Function
// =============================================================================

void applyRaycasting(HeightMap& map, const PointCloud& cloud_in_map,
                     const Eigen::Vector3f& sensor_origin_in_map,
                     const config::Raycasting& config) {
  if (!config.enabled || cloud_in_map.empty()) {
    return;
  }

  // 1. Initialize layers
  if (!map.exists(layer::raycasting_upper_bound)) {
    map.add(layer::raycasting_upper_bound, NAN);
  }
  // Reset upper bound each scan for visualization
  map.get(layer::raycasting_upper_bound).setConstant(NAN);

  if (!map.exists(layer::conflict_count)) {
    map.add(layer::conflict_count, 0.0f);
  }

  // 2. Get sensor index
  grid_map::Index sensor_idx;
  if (!map.getIndex(grid_map::Position(sensor_origin_in_map.x(),
                                       sensor_origin_in_map.y()),
                    sensor_idx)) {
    spdlog::warn("[Raycasting] Sensor origin outside map bounds");
    return;
  }

  // 3. Filter to downward rays only (point below sensor)
  // All downward rays are used for maximum coverage
  PointCloud downward_cloud;
  downward_cloud.reserve(cloud_in_map.size());
  const auto& points = cloud_in_map.xyz();

  for (const auto& point : points) {
    // Only downward rays (point below sensor height)
    if (point.z() < sensor_origin_in_map.z()) {
      downward_cloud.add(point);
    }
  }

  if (downward_cloud.empty()) {
    return;
  }

  // 4. Extract unique targets and hit mask
  auto [targets, hit_mask, cols] = extractTargets(map, downward_cloud);
  if (targets.empty()) {
    return;
  }

  // 5. Compute raycasting_upper_bound layer (max ray height per cell)
  computeUpperBounds(map, sensor_origin_in_map, sensor_idx, targets, config);

  // 6. Process conflicts: elevation > upper_bound + threshold
  size_t cells_cleared = processConflicts(map, hit_mask, config, cols);

  // 7. Reset conflict_count at target cells (measurement = not ghost)
  for (const auto& target : targets) {
    map.at(layer::conflict_count, target.index) = 0.0f;
  }

  if (cells_cleared > 0) {
    spdlog::debug("[Raycasting] Cleared {} ghost cells", cells_cleared);
  }
}

}  // namespace height_mapping::ops
