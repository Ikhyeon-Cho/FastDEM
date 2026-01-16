/*
 * raycasting.cpp
 *
 * Cell-based raycasting with Supercover algorithm for ghost obstacle removal.
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
  float height;
};

struct TargetExtraction {
  std::vector<RayTarget> targets;
  std::vector<float> hit_mask;  // NaN = no hit, value = hit height
  int cols;                     // For linear index calculation
};

TargetExtraction extractTargets(const HeightMap& map, const PointCloud& cloud) {
  const auto& size = map.getSize();
  const int rows = size(0);
  const int cols = size(1);

  // Hit mask: NaN = not visited (no hit protection)
  std::vector<float> hit_mask(rows * cols,
                              std::numeric_limits<float>::quiet_NaN());
  std::vector<grid_map::Index> active_indices;
  active_indices.reserve(cloud.size());

  for (const auto& point : cloud) {
    grid_map::Index idx;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      continue;
    }

    const int linear_idx = idx(0) * cols + idx(1);
    float& min_h = hit_mask[linear_idx];

    if (std::isnan(min_h)) {
      min_h = point.z();
      active_indices.push_back(idx);
    } else if (point.z() < min_h) {
      min_h = point.z();
    }
  }

  // Build targets from active indices
  std::vector<RayTarget> targets;
  targets.reserve(active_indices.size());
  for (const auto& idx : active_indices) {
    const int linear_idx = idx(0) * cols + idx(1);
    targets.push_back({idx, hit_mask[linear_idx]});
  }

  return {std::move(targets), std::move(hit_mask), cols};
}

void traceRaySupercover(HeightMap& map, const grid_map::Index& start,
                        const grid_map::Index& end, float z_start, float z_end,
                        const config::Raycasting& config,
                        const std::vector<float>& hit_mask, int cols,
                        size_t& cells_corrected) {
  int x = start.x();
  int y = start.y();
  const int x1 = end.x();
  const int y1 = end.y();

  int dx = x1 - x;
  int dy = y1 - y;
  const int sx = (dx > 0) ? 1 : (dx < 0) ? -1 : 0;
  const int sy = (dy > 0) ? 1 : (dy < 0) ? -1 : 0;
  dx = std::abs(dx);
  dy = std::abs(dy);

  // Total cells for true supercover = 1 + dx + dy
  const int total_cells = 1 + dx + dy;
  if (total_cells <= 1) return;

  // Height interpolation over all cells (excluding endpoint)
  const float dz = (z_end - z_start) / static_cast<float>(total_cells - 1);
  float current_z = z_start;

  // Bresenham error term (doubled for integer arithmetic)
  int err = dx - dy;
  dx *= 2;
  dy *= 2;

  // Visit all cells except endpoint (total_cells - 1 iterations)
  // Note: isValid check removed - start/end are inside map, so all
  // intermediate cells are guaranteed inside (Bresenham convexity)
  for (int i = 0; i < total_cells - 1; ++i) {
    const grid_map::Index cur_idx(x, y);
    const int linear_idx = cur_idx(0) * cols + cur_idx(1);

    // Apply Miss only if cell is NOT protected (no Hit in current scan)
    if (std::isnan(hit_mask[linear_idx])) {
      float& map_z = map.at(layer::elevation, cur_idx);
      float& persist = map.at(layer::persistence, cur_idx);

      // If map height is above ray (potential ghost)
      if (!std::isnan(map_z) && map_z > current_z + config.threshold) {
        // Decrement persistence (Miss)
        persist -= config.persistence.step_sub;

        // Clear only when persistence depleted
        if (persist <= 0.0f) {
          map_z = NAN;
          map.at(layer::variance, cur_idx) = HeightMap::initial_variance;
          map.at(layer::count, cur_idx) = 0.0f;
          persist = 0.0f;
          cells_corrected++;
        }
      }
    }

    // Move to next cell - ONE AXIS AT A TIME (true supercover)
    if (err > 0) {
      x += sx;
      err -= dy;
    } else {
      y += sy;
      err += dx;
    }

    current_z += dz;
  }
}

}  // namespace

// =============================================================================
// Main Function
// =============================================================================

void applyRaycasting(HeightMap& map, const PointCloud& scan,
                     const Eigen::Vector3f& sensor_origin,
                     const config::Raycasting& config) {
  if (!config.enabled || scan.empty()) {
    return;
  }

  // Get sensor index
  grid_map::Index sensor_idx;
  if (!map.getIndex(grid_map::Position(sensor_origin.x(), sensor_origin.y()),
                    sensor_idx)) {
    spdlog::warn("[Raycasting] Sensor origin outside map bounds");
    return;
  }

  // Extract unique targets and hit mask for protection
  auto [targets, hit_mask, cols] = extractTargets(map, scan);
  if (targets.empty()) {
    return;
  }

  const float min_dist_sq = config.min_distance * config.min_distance;
  size_t cells_corrected = 0;
  size_t rays_traced = 0;

  for (const auto& target : targets) {
    // Skip if target is same cell as sensor
    if (target.index.x() == sensor_idx.x() &&
        target.index.y() == sensor_idx.y()) {
      continue;
    }

    // Get target world position for distance check
    grid_map::Position target_pos;
    map.getPosition(target.index, target_pos);

    const float dist_sq = (target_pos.x() - sensor_origin.x()) *
                              (target_pos.x() - sensor_origin.x()) +
                          (target_pos.y() - sensor_origin.y()) *
                              (target_pos.y() - sensor_origin.y());

    // Skip short rays (division by zero protection)
    if (dist_sq < min_dist_sq) {
      continue;
    }

    // Miss: decrement persistence along ray path (with hit protection)
    traceRaySupercover(map, sensor_idx, target.index, sensor_origin.z(),
                       target.height, config, hit_mask, cols, cells_corrected);

    // Hit: increment persistence at target cell (observation)
    float& persist = map.at(layer::persistence, target.index);
    if (std::isnan(persist)) {
      persist = config.persistence.step_add;  // New cell
    } else {
      persist = std::min(persist + config.persistence.step_add,
                         config.persistence.max_count);
    }

    rays_traced++;
  }

  if (cells_corrected > 0) {
    spdlog::debug("[Raycasting] Traced {} rays, corrected {} cells",
                  rays_traced, cells_corrected);
  }
}

}  // namespace height_mapping::ops
