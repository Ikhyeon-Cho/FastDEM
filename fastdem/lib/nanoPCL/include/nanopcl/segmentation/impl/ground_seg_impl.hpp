// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEGMENTATION_IMPL_GROUND_SEG_IMPL_HPP
#define NANOPCL_SEGMENTATION_IMPL_GROUND_SEG_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <map>

namespace nanopcl {
namespace segmentation {

namespace detail {

struct CellData {
  std::vector<size_t> point_indices;
  std::vector<float> z_values;
  float robust_min_z = 0;
  bool is_obstacle_only = false;
};

using CellKey = std::pair<int, int>;
using CellMap = std::map<CellKey, CellData>;

inline CellMap groupPointsByGrid(const PointCloud& cloud, float resolution) {
  CellMap cells;
  const auto& pts = cloud.points();

  for (size_t i = 0; i < pts.size(); ++i) {
    const auto& pt = pts[i];
    int gx = static_cast<int>(std::floor(pt.x() / resolution));
    int gy = static_cast<int>(std::floor(pt.y() / resolution));

    auto& cell = cells[{gx, gy}];
    cell.point_indices.push_back(i);
    cell.z_values.push_back(pt.z());
  }

  return cells;
}

inline void processCells(CellMap& cells, const GroundSegConfig& config) {
  for (auto& [key, cell] : cells) {
    if (cell.z_values.size() < config.min_points_per_cell) {
      cell.is_obstacle_only = true;
      continue;
    }

    // Sort for percentile calculation
    std::sort(cell.z_values.begin(), cell.z_values.end());

    size_t idx =
        static_cast<size_t>(config.cell_percentile * (cell.z_values.size() - 1));
    idx = std::min(idx, cell.z_values.size() - 1);
    cell.robust_min_z = cell.z_values[idx];

    // Check if cell is too high to be ground
    if (cell.robust_min_z > config.max_ground_height) {
      cell.is_obstacle_only = true;
    }
  }
}

} // namespace detail

inline GroundSegResult segmentGround(const PointCloud& cloud,
                                     const GroundSegConfig& config) {
  GroundSegResult result;

  if (cloud.empty()) {
    return result;
  }

  // Group points by grid cells and process
  auto cells = detail::groupPointsByGrid(cloud, config.grid_resolution);
  detail::processCells(cells, config);

  // Reserve estimated capacity
  const size_t n = cloud.size();
  result.ground.reserve(n / 2);
  result.obstacles.reserve(n / 2);

  // Classify points
  const auto& pts = cloud.points();
  for (size_t i = 0; i < n; ++i) {
    const auto& pt = pts[i];
    int gx = static_cast<int>(std::floor(pt.x() / config.grid_resolution));
    int gy = static_cast<int>(std::floor(pt.y() / config.grid_resolution));

    auto it = cells.find({gx, gy});
    if (it == cells.end()) {
      continue;
    }

    const auto& cell = it->second;

    // Obstacle-only cell or above ground threshold
    bool is_obstacle =
        cell.is_obstacle_only ||
        (pt.z() > cell.robust_min_z + config.ground_thickness);

    if (is_obstacle) {
      result.obstacles.push_back(static_cast<uint32_t>(i));
    } else {
      result.ground.push_back(static_cast<uint32_t>(i));
    }
  }

  return result;
}

inline GroundSegResult segmentGround(const PointCloud& cloud,
                                     float grid_resolution,
                                     float ground_thickness,
                                     float max_ground_height) {
  GroundSegConfig config;
  config.grid_resolution = grid_resolution;
  config.ground_thickness = ground_thickness;
  config.max_ground_height = max_ground_height;
  return segmentGround(cloud, config);
}

} // namespace segmentation
} // namespace nanopcl

#endif // NANOPCL_SEGMENTATION_IMPL_GROUND_SEG_IMPL_HPP
