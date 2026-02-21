// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * raycasting.cpp
 *
 * Ghost obstacle removal via log-odds free-space accumulation.
 *
 * Algorithm:
 * 1. Process scan (single pass over points):
 *    - Points inside map → logodds += L_observed
 *    - Trace ray from sensor to target → min ray height per traversed cell
 * 2. Resolve ghost cells (pass over ray-traversed cells only):
 *    - elevation > min_ray_height + threshold → logodds -= L_ghost
 *    - logodds < clear_threshold → clear cell
 *
 * Using min ray height: if ANY ray passes below recorded elevation,
 * that ray physically penetrated the recorded obstacle → ghost evidence.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/postprocess/raycasting.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <vector>

namespace fastdem {

namespace {

constexpr float kMinRayLength = 1e-4f;
constexpr float kSampleStepRatio = 0.5f;  // Sample at half resolution

/**
 * @brief Trace ray, tracking min height per cell and collecting new cells.
 *
 * Cells visited for the first time (NaN → value) are appended to ray_cells.
 */
void traceRay(const ElevationMap& map, float resolution,
              const Eigen::Vector3f& start, const Eigen::Vector3f& end,
              grid_map::Matrix& ray_min_mat,
              std::vector<grid_map::Index>& ray_cells) {
  const Eigen::Vector3f ray = end - start;
  const float ray_length_2d = std::sqrt(ray.x() * ray.x() + ray.y() * ray.y());

  if (ray_length_2d < kMinRayLength) return;

  const Eigen::Vector3f dir = ray / ray_length_2d;
  const float step = resolution * kSampleStepRatio;

  for (float t = 0.0f; t < ray_length_2d; t += step) {
    const Eigen::Vector3f sample = start + dir * t;

    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(sample.x(), sample.y()), idx)) {
      continue;
    }

    float& current_min = ray_min_mat(idx(0), idx(1));
    if (std::isnan(current_min)) {
      current_min = sample.z();
      ray_cells.emplace_back(idx);
    } else if (sample.z() < current_min) {
      current_min = sample.z();
    }
  }
}

/**
 * @brief Process scan: observed evidence + ray tracing in a single pass.
 *
 * For each point:
 * - Inside map → logodds += L_observed (cell is alive)
 * - Downward ray → trace to target, update min height, collect traversed cells
 *
 * @return Unique cell indices traversed by rays.
 */
std::vector<grid_map::Index> processScan(ElevationMap& map,
                                         const PointCloud& scan,
                                         const Eigen::Vector3f& sensor_origin,
                                         const config::Raycasting& config) {
  auto& logodds_mat = map.get(layer::visibility_logodds);
  auto& min_height_mat = map.get(layer::raycasting);
  const float resolution = map.getResolution();

  std::vector<grid_map::Index> ray_cells;

  for (size_t i : scan.indices()) {
    const Eigen::Vector3f pt = scan.point(i);

    // Observed evidence: point inside map → cell is alive
    grid_map::Index idx;
    if (map.getIndex(grid_map::Position(pt.x(), pt.y()), idx)) {
      float& logodds = logodds_mat(idx(0), idx(1));
      if (std::isnan(logodds)) logodds = 0.0f;
      logodds =
          std::min(logodds + config.log_odds_observed, config.log_odds_max);
    }

    // Ray tracing: skip upward rays
    if (pt.z() >= sensor_origin.z()) continue;

    traceRay(map, resolution, sensor_origin, pt, min_height_mat, ray_cells);
  }

  return ray_cells;
}

/**
 * @brief Resolve ghost cells from ray-traversed cells.
 *
 * Only visits cells where rays actually passed. If a ray passed below the
 * recorded elevation, that's ghost evidence (logodds decreases). Cells whose
 * logodds fall below clear_threshold are cleared.
 */
void resolveGhostCells(ElevationMap& map,
                       const std::vector<grid_map::Index>& ray_cells,
                       const config::Raycasting& config) {
  const auto& min_height_mat = map.get(layer::raycasting);
  const auto& elevation_mat = map.get(layer::elevation);
  auto& logodds_mat = map.get(layer::visibility_logodds);

  for (const auto& idx : ray_cells) {
    const int i = idx(0);
    const int j = idx(1);

    if (std::isnan(elevation_mat(i, j))) continue;

    // Conflict: a ray physically passed below the recorded elevation
    if (elevation_mat(i, j) >
        min_height_mat(i, j) + config.height_conflict_threshold) {
      float& logodds = logodds_mat(i, j);
      if (std::isnan(logodds)) logodds = 0.0f;
      logodds -= config.log_odds_ghost;

      if (logodds < config.clear_threshold) {
        map.clearAt(idx);
        map.at(layer::ghost_removal, idx) = 1.0f;
      }
    }
  }
}

}  // namespace

void applyRaycasting(ElevationMap& map, const PointCloud& scan,
                     const Eigen::Vector3f& sensor_origin,
                     const config::Raycasting& config) {
  if (!config.enabled || scan.empty()) {
    return;
  }

  // 1. Validate preconditions
  if (!map.exists(layer::elevation)) {
    spdlog::warn("[Raycasting] Missing required layer: elevation.");
    return;
  }
  if (!map.isInside(grid_map::Position(sensor_origin.x(), sensor_origin.y()))) {
    spdlog::warn("[Raycasting] Sensor origin outside map bounds");
    return;
  }

  // 2. Initialize layers
  if (!map.exists(layer::ghost_removal)) map.add(layer::ghost_removal);
  if (!map.exists(layer::raycasting)) map.add(layer::raycasting);
  if (!map.exists(layer::visibility_logodds))
    map.add(layer::visibility_logodds);

  // Reset per-frame internal layer
  map.clear(layer::raycasting);

  // 3. Process scan: observed evidence + ray tracing
  auto ray_cells = processScan(map, scan, sensor_origin, config);

  // 4. Resolve ghost cells
  resolveGhostCells(map, ray_cells, config);
}

}  // namespace fastdem
