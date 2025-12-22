/*
 * grid_ground_segmentation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/algorithms/algorithms.h"

#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace height_mapping::algorithms {

namespace {

struct CellData {
  std::vector<size_t> point_indices;
  std::vector<float> z_values;
  float robust_min_z = 0;
};

std::map<std::pair<int, int>, CellData> groupPointsByGrid(
    const PointCloud& cloud, const GridGroundSegmentationConfig& config) {
  std::map<std::pair<int, int>, CellData> cells_data;

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud.points[i];

    // Hash point into grid cell
    int grid_x =
        static_cast<int>(std::floor(point.x() / config.grid_resolution));
    int grid_y =
        static_cast<int>(std::floor(point.y() / config.grid_resolution));

    auto cell_key = std::make_pair(grid_x, grid_y);
    cells_data[cell_key].point_indices.push_back(i);
    cells_data[cell_key].z_values.push_back(point.z());
  }

  return cells_data;
}

void processCells(std::map<std::pair<int, int>, CellData>& cells_data,
                  const GridGroundSegmentationConfig& config,
                  size_t& processed_cells, size_t& obstacle_only_cells) {
  for (auto& [cell_key, cell_data] : cells_data) {
    // Skip cells with too few points
    if (cell_data.z_values.size() < config.min_points_per_cell) {
      continue;
    }

    // Sort z-values for percentile calculation
    std::sort(cell_data.z_values.begin(), cell_data.z_values.end());

    // Use percentile for robust minimum (ignores outliers)
    size_t percentile_idx = static_cast<size_t>(
        config.cell_percentile * (cell_data.z_values.size() - 1));
    percentile_idx = std::min(percentile_idx, cell_data.z_values.size() - 1);

    cell_data.robust_min_z = cell_data.z_values[percentile_idx];

    // Check if this cell's minimum is too high to be ground
    // This handles obstacle-only cells (e.g., walls, poles)
    if (cell_data.robust_min_z > config.max_ground_height) {
      // Mark as non-ground cell
      cell_data.robust_min_z = std::numeric_limits<float>::max();
      obstacle_only_cells++;
    } else {
      processed_cells++;
    }
  }
}

PointCloud classifyAndFilter(
    const PointCloud& cloud,
    const std::map<std::pair<int, int>, CellData>& cells_data,
    const GridGroundSegmentationConfig& config,
    size_t& ground_count, size_t& obstacle_count) {
  PointCloud filtered_cloud;
  filtered_cloud.reserve(cloud.size());

  // Enable channels if present in input
  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();
  if (has_intensity) filtered_cloud.enableIntensity();
  if (has_color) filtered_cloud.enableColor();

  // Lambda to add a point with its sensor data
  auto addPointWithData = [&](size_t idx) {
    filtered_cloud.points.push_back(cloud.points[idx]);
    if (has_intensity)
      filtered_cloud.intensity().push_back(cloud.intensity()[idx]);
    if (has_color) filtered_cloud.color().push_back(cloud.color()[idx]);
  };

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud.points[i];

    // Find which cell this point belongs to
    int grid_x =
        static_cast<int>(std::floor(point.x() / config.grid_resolution));
    int grid_y =
        static_cast<int>(std::floor(point.y() / config.grid_resolution));
    auto cell_key = std::make_pair(grid_x, grid_y);

    auto it = cells_data.find(cell_key);
    if (it == cells_data.end() ||
        it->second.z_values.size() < config.min_points_per_cell) {
      // Cell not processed (too few points)
      continue;
    }

    // Check if this is an obstacle-only cell
    if (it->second.robust_min_z == std::numeric_limits<float>::max()) {
      // All points in this cell are obstacles
      obstacle_count++;
      if (!config.keep_only_ground) {
        addPointWithData(i);
      }
      continue;
    }

    // Check if point is within ground thickness of cell minimum
    float cell_ground_threshold =
        it->second.robust_min_z + config.ground_thickness;

    if (point.z() <= cell_ground_threshold) {
      // Ground point
      ground_count++;
      // If we keep ground, add it. Or if we keep obstacles, we skip this?
      // Logic: if keep_only_ground, we add ground. if !keep_only_ground, we add both.
      // So in both cases we add ground? No, usually keep_only_ground filters OUT obstacles.
      // Wait, keep_only_ground=true means OUTPUT IS GROUND ONLY.
      // keep_only_ground=false means OUTPUT IS EVERYTHING (segmentation only updates labels?)
      // Here we are creating a filtered cloud.
      
      // If keep_only_ground is true: add ground points.
      // If keep_only_ground is false: add ground points (and obstacles later).
      // So ground points are always added unless we have a "remove ground" mode.
      addPointWithData(i);
    } else {
      // Obstacle point
      obstacle_count++;
      if (!config.keep_only_ground) {
        addPointWithData(i);
      }
    }
  }

  return filtered_cloud;
}

}  // namespace

void segmentGroundGrid(PointCloud& cloud,
                       const GridGroundSegmentationConfig& config) {
  if (cloud.empty()) {
    return;
  }

  // Group points by grid cells
  auto cells_data = groupPointsByGrid(cloud, config);

  // Process cells to find robust ground level
  size_t processed_cells = 0;
  size_t obstacle_only_cells = 0;
  processCells(cells_data, config, processed_cells, obstacle_only_cells);

  // Classify and filter points based on cell ground levels
  size_t ground_count = 0;
  size_t obstacle_count = 0;
  auto filtered = classifyAndFilter(cloud, cells_data, config, ground_count, obstacle_count);

  // Preserve metadata
  filtered.setTimestamp(cloud.timestamp());
  filtered.setFrameId(cloud.frameId());

  // Swap to avoid copy
  cloud = std::move(filtered);

  spdlog::debug(
      "[GridSeg] {} processed cells ({} obs-only). Points: {} ground, {} obstacle.",
      processed_cells, obstacle_only_cells, ground_count, obstacle_count);
}

}  // namespace height_mapping::algorithms