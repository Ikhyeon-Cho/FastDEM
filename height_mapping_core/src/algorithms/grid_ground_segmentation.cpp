/*
 * grid_ground_segmentation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/algorithms/grid_ground_segmentation.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace height_mapping::algorithms {

GridGroundSegmentation::Stats GridGroundSegmentation::segmentInPlace(
    height_mapping::PointCloud &cloud) const {
  Stats stats;

  if (cloud.empty()) {
    return stats;
  }

  // Group points by grid cells
  auto cells_data = groupPointsByGrid(cloud);

  // Process cells to find robust ground level
  processCells(cells_data, stats);

  // Classify and filter points based on cell ground levels
  auto filtered = classifyAndFilter(cloud, cells_data, stats);

  // Preserve metadata
  filtered.setTimestamp(cloud.timestamp());
  filtered.setFrameId(cloud.frameId());

  // Swap to avoid copy
  cloud = std::move(filtered);

  return stats;
}

height_mapping::PointCloud
GridGroundSegmentation::segment(const height_mapping::PointCloud &cloud,
                                Stats &stats) const {
  height_mapping::PointCloud result = cloud; // Copy
  stats = segmentInPlace(result);            // Modify copy
  return result;
}

std::map<std::pair<int, int>, GridGroundSegmentation::CellData>
GridGroundSegmentation::groupPointsByGrid(
    const height_mapping::PointCloud &cloud) const {
  std::map<std::pair<int, int>, CellData> cells_data;

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto point = cloud[i];

    // Hash point into grid cell
    int grid_x =
        static_cast<int>(std::floor(point.x() / config_.grid_resolution));
    int grid_y =
        static_cast<int>(std::floor(point.y() / config_.grid_resolution));

    auto cell_key = std::make_pair(grid_x, grid_y);
    cells_data[cell_key].point_indices.push_back(i);
    cells_data[cell_key].z_values.push_back(point.z());
  }

  return cells_data;
}

void GridGroundSegmentation::processCells(
    std::map<std::pair<int, int>, CellData> &cells_data, Stats &stats) const {

  stats.total_cells = cells_data.size();

  for (auto &[cell_key, cell_data] : cells_data) {
    // Skip cells with too few points
    if (cell_data.z_values.size() < config_.min_points_per_cell) {
      continue;
    }

    // Sort z-values for percentile calculation
    std::sort(cell_data.z_values.begin(), cell_data.z_values.end());

    // Use percentile for robust minimum (ignores outliers)
    size_t percentile_idx = static_cast<size_t>(
        config_.cell_percentile * (cell_data.z_values.size() - 1));
    percentile_idx = std::min(percentile_idx, cell_data.z_values.size() - 1);

    cell_data.robust_min_z = cell_data.z_values[percentile_idx];

    // Check if this cell's minimum is too high to be ground
    // This handles obstacle-only cells (e.g., walls, poles)
    if (cell_data.robust_min_z > config_.max_ground_height) {
      // Mark as non-ground cell
      cell_data.robust_min_z = std::numeric_limits<float>::max();
      stats.obstacle_only_cells++;
    } else {
      stats.processed_cells++;
    }
  }
}

height_mapping::PointCloud GridGroundSegmentation::classifyAndFilter(
    const height_mapping::PointCloud &cloud,
    const std::map<std::pair<int, int>, CellData> &cells_data,
    Stats &stats) const {

  height_mapping::PointCloud filtered_cloud;
  filtered_cloud.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto point = cloud[i];

    // Find which cell this point belongs to
    int grid_x =
        static_cast<int>(std::floor(point.x() / config_.grid_resolution));
    int grid_y =
        static_cast<int>(std::floor(point.y() / config_.grid_resolution));
    auto cell_key = std::make_pair(grid_x, grid_y);

    auto it = cells_data.find(cell_key);
    if (it == cells_data.end() ||
        it->second.z_values.size() < config_.min_points_per_cell) {
      // Cell not processed (too few points)
      stats.unprocessed_count++;
      continue;
    }

    // Check if this is an obstacle-only cell
    if (it->second.robust_min_z == std::numeric_limits<float>::max()) {
      // All points in this cell are obstacles
      stats.obstacle_count++;
      if (!config_.keep_only_ground) {
        filtered_cloud.addPoint(point.xyz(), point.intensity(), point.color());
      }
      continue;
    }

    // Check if point is within ground thickness of cell minimum
    float cell_ground_threshold =
        it->second.robust_min_z + config_.ground_thickness;

    if (point.z() <= cell_ground_threshold) {
      // Ground point
      stats.ground_count++;
      if (config_.keep_only_ground) {
        filtered_cloud.addPoint(point.xyz(), point.intensity(), point.color());
      }
    } else {
      // Obstacle point
      stats.obstacle_count++;
      if (!config_.keep_only_ground) {
        filtered_cloud.addPoint(point.xyz(), point.intensity(), point.color());
      }
    }
  }

  return filtered_cloud;
}

} // namespace height_mapping::algorithms