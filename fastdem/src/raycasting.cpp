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

/**
 * @brief Trace ray using DDA grid traversal.
 *
 * Visits exactly the cells the ray passes through (no redundant samples).
 * Cells visited for the first time (NaN → value) are appended to ray_cells.
 */
void traceRay(const ElevationMap& map, float resolution,
              const Eigen::Vector3f& start, const Eigen::Vector3f& end,
              grid_map::Matrix& ray_min_mat,
              std::vector<grid_map::Index>& ray_cells) {
  const float dx = end.x() - start.x();
  const float dy = end.y() - start.y();
  const float ray_len_2d = std::sqrt(dx * dx + dy * dy);
  if (ray_len_2d < kMinRayLength) return;

  const float dz = end.z() - start.z();

  // Grid properties
  const auto map_center = map.getPosition();
  const int nrows = map.getSize()(0);
  const int ncols = map.getSize()(1);
  const auto& buf_start = map.getStartIndex();

  // grid_map coordinate mapping:
  //   row = (center.x + length.x/2 - pos.x) / resolution
  //   col = (center.y + length.y/2 - pos.y) / resolution
  const float origin_x =
      static_cast<float>(map_center.x()) + nrows * resolution * 0.5f;
  const float origin_y =
      static_cast<float>(map_center.y()) + ncols * resolution * 0.5f;

  const float gr0 = (origin_x - start.x()) / resolution;
  const float gc0 = (origin_y - start.y()) / resolution;
  const float gr1 = (origin_x - end.x()) / resolution;
  const float gc1 = (origin_y - end.y()) / resolution;

  const float dr = gr1 - gr0;
  const float dc = gc1 - gc0;

  int r = static_cast<int>(std::floor(gr0));
  int c = static_cast<int>(std::floor(gc0));

  // DDA setup (t parameterized in [0, 1]: t=0 at start, t=1 at end)
  int step_r, step_c;
  float t_max_r, t_max_c, t_delta_r, t_delta_c;

  if (std::abs(dr) > 1e-8f) {
    step_r = (dr > 0) ? 1 : -1;
    float boundary = (step_r > 0) ? (r + 1.0f) : static_cast<float>(r);
    t_max_r = (boundary - gr0) / dr;
    t_delta_r = static_cast<float>(step_r) / dr;
  } else {
    step_r = 0;
    t_max_r = 1e30f;
    t_delta_r = 1e30f;
  }

  if (std::abs(dc) > 1e-8f) {
    step_c = (dc > 0) ? 1 : -1;
    float boundary = (step_c > 0) ? (c + 1.0f) : static_cast<float>(c);
    t_max_c = (boundary - gc0) / dc;
    t_delta_c = static_cast<float>(step_c) / dc;
  } else {
    step_c = 0;
    t_max_c = 1e30f;
    t_delta_c = 1e30f;
  }

  // Traverse ray cells
  const int max_steps = nrows + ncols;
  for (int s = 0; s < max_steps; ++s) {
    if (r >= 0 && r < nrows && c >= 0 && c < ncols) {
      const int mr = (r + buf_start(0)) % nrows;
      const int mc = (c + buf_start(1)) % ncols;

      // Ray height at cell exit (min height for downward rays)
      const float t_exit = std::min(t_max_r, t_max_c);
      const float height = start.z() + std::min(t_exit, 1.0f) * dz;

      float& cur_min = ray_min_mat(mr, mc);
      if (std::isnan(cur_min)) {
        cur_min = height;
        ray_cells.emplace_back(grid_map::Index(mr, mc));
      } else if (height < cur_min) {
        cur_min = height;
      }
    }

    // Advance to next cell
    if (t_max_r < t_max_c) {
      if (t_max_r >= 1.0f) break;
      r += step_r;
      t_max_r += t_delta_r;
    } else {
      if (t_max_c >= 1.0f) break;
      c += step_c;
      t_max_c += t_delta_c;
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
