// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * raycasting.cpp
 *
 * Raycasting for ghost obstacle removal using maximum ray height.
 *
 * Algorithm:
 * 1. Filter downward rays (point.z < sensor.z)
 * 2. Extract targets (min height per cell)
 * 3. Compute raycasting_upper_bound layer (max ray height at each cell)
 * 4. Conflict detection: elevation > raycasting_upper_bound + threshold
 * 5. Temporal voting: increment conflict_count, clear when >= vote_threshold
 *
 * Key insight: Using maximum ray height ensures conservative ghost detection.
 * "All rays passed below the recorded elevation" = confirmed ghost.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/postprocess/raycasting.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>
#include <nanopcl/filters/crop.hpp>
#include <unordered_map>
#include <vector>

namespace fastdem {

namespace {

// Constants
constexpr float kMinRayLength = 1e-4f;
constexpr float kSampleStepRatio = 0.5f;  // Sample at half resolution

struct RayTarget {
  grid_map::Index index;
  Eigen::Vector3f point;
};

/**
 * @brief Extract ray targets from point cloud.
 *
 * For each cell, keeps the minimum height point as the target.
 *
 * @param map Height map for coordinate conversion
 * @param cloud Point cloud (already filtered for downward rays)
 * @return Vector of ray targets
 */
std::vector<RayTarget> extractTargets(const ElevationMap& map,
                                      const PointCloud& cloud) {
  const int cols = map.getSize()(1);
  std::unordered_map<int, RayTarget> cell_map;
  cell_map.reserve(cloud.size());

  for (size_t i : cloud.indices()) {
    Eigen::Vector3f point = cloud.point(i);
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) continue;

    const int linear_idx = idx(0) * cols + idx(1);
    auto [it, inserted] =
        cell_map.try_emplace(linear_idx, RayTarget{idx, point});
    if (!inserted && point.z() < it->second.point.z()) {
      it->second.point = point;
    }
  }

  std::vector<RayTarget> targets;
  targets.reserve(cell_map.size());
  for (auto& [_, t] : cell_map) {
    targets.push_back(std::move(t));
  }
  return targets;
}

/**
 * @brief Trace ray by sampling, tracking maximum height at each cell.
 *
 * Samples points along the ray at intervals smaller than grid resolution,
 * ensuring all cells the ray passes through are visited (supercover effect).
 * Tracks maximum ray height at each cell for conservative ghost detection.
 *
 * @param map Height map (for coordinate conversion)
 * @param resolution Grid resolution (cached)
 * @param start 3D ray start position (sensor)
 * @param end 3D ray end position (target)
 * @param ray_max_mat Matrix tracking maximum ray height per cell
 * @param endpoint_margin Number of cells to skip before target
 */
void traceRayForMaxHeight(const ElevationMap& map, float resolution,
                          const Eigen::Vector3f& start,
                          const Eigen::Vector3f& end,
                          grid_map::Matrix& ray_max_mat, int endpoint_margin) {
  const Eigen::Vector3f ray = end - start;
  const float ray_length_2d = std::sqrt(ray.x() * ray.x() + ray.y() * ray.y());

  if (ray_length_2d < kMinRayLength) return;

  const Eigen::Vector3f dir = ray / ray_length_2d;
  const float step = resolution * kSampleStepRatio;
  const float trace_end = ray_length_2d - endpoint_margin * resolution;

  if (trace_end <= 0.0f) return;

  for (float t = 0.0f; t < trace_end; t += step) {
    const Eigen::Vector3f sample = start + dir * t;

    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(sample.x(), sample.y()), idx)) {
      continue;
    }

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
 * @param targets Ray targets
 * @param config Raycasting configuration
 */
void computeUpperBounds(ElevationMap& map, const Eigen::Vector3f& sensor_origin,
                        const std::vector<RayTarget>& targets,
                        const config::Raycasting& config) {
  auto& upper_bound_mat = map.get(layer::raycasting_upper_bound);
  const float resolution = map.getResolution();

  grid_map::Index sensor_idx;
  map.getIndex(grid_map::Position(sensor_origin.x(), sensor_origin.y()),
               sensor_idx);

  for (const auto& target : targets) {
    if (target.index(0) == sensor_idx(0) && target.index(1) == sensor_idx(1)) {
      continue;
    }

    traceRayForMaxHeight(map, resolution, sensor_origin, target.point,
                         upper_bound_mat, config.endpoint_margin);
  }
}

/**
 * @brief Detect ghost cells based on upper_bound vs elevation comparison.
 *
 * Conflict condition: elevation > raycasting_upper_bound + threshold
 * (all rays passed below the recorded elevation = ghost obstacle)
 *
 * Only checks cells where elevation_max - elevation_min > dynamic_height_threshold,
 * which filters out static obstacles (walls, etc.) that have consistent height.
 *
 * @param upper_bound Raycasting upper bound layer
 * @param elevation Elevation layer
 * @param elevation_min Minimum elevation layer
 * @param elevation_max Maximum elevation layer
 * @param conflict_count Conflict count layer (modified in place)
 * @param config Raycasting configuration
 * @return Indices of cells to clear
 */
std::vector<grid_map::Index> detectGhostCells(
    const grid_map::Matrix& upper_bound,    //
    const grid_map::Matrix& elevation,      //
    const grid_map::Matrix& elevation_min,  //
    const grid_map::Matrix& elevation_max,  //
    grid_map::Matrix& conflict_count,       //
    const config::Raycasting& config) {
  const int rows = upper_bound.rows();
  const int cols = upper_bound.cols();

  std::vector<grid_map::Index> ghost_cells;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      // Skip: no ray passed or no elevation
      if (std::isnan(upper_bound(i, j))) continue;
      if (std::isnan(elevation(i, j))) continue;

      // Skip: static obstacle (small height variation = consistent measurement)
      const float height_diff = elevation_max(i, j) - elevation_min(i, j);
      if (std::isnan(height_diff) ||
          height_diff < config.dynamic_height_threshold) {
        continue;
      }

      // Conflict: elevation > upper_bound + threshold
      if (elevation(i, j) > upper_bound(i, j) + config.ray_height_margin) {
        // Initialize if NaN (new cell from map shift)
        if (std::isnan(conflict_count(i, j))) {
          conflict_count(i, j) = 0.0f;
        }
        conflict_count(i, j) += 1.0f;

        if (conflict_count(i, j) >= static_cast<float>(config.vote_threshold)) {
          ghost_cells.emplace_back(i, j);
        }
      }
    }
  }

  return ghost_cells;
}

}  // namespace

void applyRaycasting(ElevationMap& map, const PointCloud& cloud_in_map,
                     const Eigen::Vector3f& sensor_position_in_map,
                     const config::Raycasting& config) {
  if (!config.enabled || cloud_in_map.empty()) {
    return;
  }

  // 1. Initialize layers
  if (!map.exists(layer::raycasting_upper_bound)) {
    map.add(layer::raycasting_upper_bound);
  }
  if (!map.exists(layer::conflict_count)) {
    map.add(layer::conflict_count);
  }
  // Reset upper bound layer
  map.clear(layer::raycasting_upper_bound);

  // 2. Validate sensor position
  if (!map.isInside(grid_map::Position(sensor_position_in_map.x(),
                                       sensor_position_in_map.y()))) {
    spdlog::warn("[Raycasting] Sensor origin outside map bounds");
    return;
  }

  // 3. Filter to downward rays only
  auto downward_cloud = nanopcl::filters::cropZ(
      cloud_in_map, -std::numeric_limits<float>::infinity(),
      sensor_position_in_map.z());
  if (downward_cloud.empty()) return;

  // 4. Extract targets
  auto ray_targets = extractTargets(map, downward_cloud);
  if (ray_targets.empty()) return;

  // 5. Compute upper bound layer
  computeUpperBounds(map, sensor_position_in_map, ray_targets, config);

  // 6. Detect and clear ghost cells
  auto ghost_cells = detectGhostCells(
      map.get(layer::raycasting_upper_bound), map.get(layer::elevation),
      map.get(layer::elevation_min), map.get(layer::elevation_max),
      map.get(layer::conflict_count), config);
  for (const auto& idx : ghost_cells) {
    map.clearAt(idx);
  }
}

}  // namespace fastdem
