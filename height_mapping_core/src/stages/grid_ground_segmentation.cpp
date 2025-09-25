/*
 * grid_ground_segmentation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/grid_ground_segmentation.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registration.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <logger/logger.h>

namespace height_mapping::core::stages {

constexpr const char *STAGE_NAME = "GridGroundSegmentation";

GridGroundSegmentation::GridGroundSegmentation() : Stage(STAGE_NAME) {}

void GridGroundSegmentation::configure(
    const std::map<std::string, std::string> &params) {

  auto getParam = [&params](const std::string &key, auto default_val) {
    auto it = params.find(key);
    if (it != params.end()) {
      if constexpr (std::is_same_v<decltype(default_val), float>) {
        return std::stof(it->second);
      } else if constexpr (std::is_same_v<decltype(default_val), size_t>) {
        return static_cast<size_t>(std::stoi(it->second));
      } else if constexpr (std::is_same_v<decltype(default_val), bool>) {
        return it->second == "true" || it->second == "1";
      }
    }
    return default_val;
  };

  grid_resolution_ = getParam("grid_resolution", 0.5f);
  cell_percentile_ = getParam("cell_percentile", 0.2f);
  ground_thickness_ = getParam("ground_thickness", 0.3f);
  max_ground_height_ = getParam("max_ground_height", 0.5f);
  min_points_per_cell_ = getParam("min_points_per_cell", size_t(2));
  keep_only_ground_ = getParam("keep_only_ground", true);

  std::string msg =
      "Configured with grid_resolution=" + std::to_string(grid_resolution_) +
      "m, cell_percentile=" + std::to_string(cell_percentile_) +
      ", ground_thickness=" + std::to_string(ground_thickness_) +
      "m, max_ground_height=" + std::to_string(max_ground_height_) +
      "m, min_points_per_cell=" + std::to_string(min_points_per_cell_) +
      ", keep_only_ground=" + (keep_only_ground_ ? "true" : "false");
  LOG_DEBUG(STAGE_NAME, msg);
}

void GridGroundSegmentation::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  auto &cloud = mapping_ctx.cloud();

  if (cloud.empty()) {
    LOG_WARN(STAGE_NAME, "Input cloud is empty, skipping segmentation...");
    return;
  }

  // Group points by grid cells (robot-centric, in base_link frame)
  std::map<std::pair<int, int>, CellData> cells_data;

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto point = cloud[i];

    // Hash point into grid cell
    int grid_x = static_cast<int>(std::floor(point.x() / grid_resolution_));
    int grid_y = static_cast<int>(std::floor(point.y() / grid_resolution_));

    auto cell_key = std::make_pair(grid_x, grid_y);
    cells_data[cell_key].point_indices.push_back(i);
    cells_data[cell_key].z_values.push_back(point.z());
  }

  // Process each cell to find robust minimum with height constraint
  size_t total_cells = 0;
  size_t processed_cells = 0;
  size_t obstacle_only_cells = 0;

  for (auto &[cell_key, cell_data] : cells_data) {
    total_cells++;

    // Skip cells with too few points
    if (cell_data.z_values.size() < min_points_per_cell_) {
      continue;
    }

    // Sort z-values for percentile calculation
    std::sort(cell_data.z_values.begin(), cell_data.z_values.end());

    // Use percentile for robust minimum (ignores outliers)
    size_t percentile_idx =
        static_cast<size_t>(cell_percentile_ * (cell_data.z_values.size() - 1));
    percentile_idx = std::min(percentile_idx, cell_data.z_values.size() - 1);

    cell_data.robust_min_z = cell_data.z_values[percentile_idx];

    // Check if this cell's minimum is too high to be ground
    // This handles obstacle-only cells (e.g., walls, poles)
    if (cell_data.robust_min_z > max_ground_height_) {
      // Mark as non-ground cell
      cell_data.robust_min_z = std::numeric_limits<float>::max();
      obstacle_only_cells++;
    } else {
      processed_cells++;
    }
  }

  // Classify and filter points based on cell-wise ground threshold
  size_t ground_count = 0;
  size_t obstacle_count = 0;
  size_t unprocessed_count = 0;

  geometry::PointCloud filtered_cloud;
  filtered_cloud.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto point = cloud[i];

    // Find which cell this point belongs to
    int grid_x = static_cast<int>(std::floor(point.x() / grid_resolution_));
    int grid_y = static_cast<int>(std::floor(point.y() / grid_resolution_));
    auto cell_key = std::make_pair(grid_x, grid_y);

    auto it = cells_data.find(cell_key);
    if (it == cells_data.end() ||
        it->second.z_values.size() < min_points_per_cell_) {
      // Cell not processed (too few points)
      unprocessed_count++;
      continue;
    }

    // Check if this is an obstacle-only cell
    if (it->second.robust_min_z == std::numeric_limits<float>::max()) {
      // All points in this cell are obstacles
      obstacle_count++;
      if (!keep_only_ground_) {
        filtered_cloud.addPoint(point.xyz(), point.intensity(), point.color());
      }
      continue;
    }

    // Check if point is within ground thickness of cell minimum
    float cell_ground_threshold = it->second.robust_min_z + ground_thickness_;

    if (point.z() <= cell_ground_threshold) {
      // Ground point
      ground_count++;
      if (keep_only_ground_) {
        filtered_cloud.addPoint(point.xyz(), point.intensity(), point.color());
      }
    } else {
      // Obstacle point
      obstacle_count++;
      if (!keep_only_ground_) {
        filtered_cloud.addPoint(point.xyz(), point.intensity(), point.color());
      }
    }
  }

  std::string msg = "Grid ground segmentation: " + std::to_string(total_cells) +
                    " total cells, " + std::to_string(processed_cells) +
                    " processed, " + std::to_string(obstacle_only_cells) +
                    " obstacle-only cells; " + std::to_string(ground_count) +
                    " ground points, " + std::to_string(obstacle_count) +
                    " obstacle points, " + std::to_string(unprocessed_count) +
                    " unprocessed points";
  LOG_DEBUG_THROTTLE(1.0, STAGE_NAME, msg);

  // Replace original cloud
  filtered_cloud.setTimestamp(cloud.timestamp());
  filtered_cloud.setFrameId(cloud.frameId());
  cloud = std::move(filtered_cloud);
}

// Register this stage with the factory
REGISTER_STAGE(GridGroundSegmentation)

} // namespace height_mapping::core::stages