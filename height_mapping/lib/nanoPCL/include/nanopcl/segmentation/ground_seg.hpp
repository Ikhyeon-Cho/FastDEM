// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Grid-based Ground Segmentation
//
// Divides point cloud into 2D grid cells and analyzes height distribution
// per cell. More robust than RANSAC for uneven terrain.
//
// Example usage:
//   auto result = segmentation::segmentGround(cloud);
//   auto ground = cloud[result.ground];
//   auto obstacles = cloud[result.obstacles];

#ifndef NANOPCL_SEGMENTATION_GROUND_SEG_HPP
#define NANOPCL_SEGMENTATION_GROUND_SEG_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace npcl {
namespace segmentation {

// =============================================================================
// Ground Segmentation Configuration
// =============================================================================

/**
 * @brief Configuration for grid-based ground segmentation
 */
struct GroundSegConfig {
  float grid_resolution = 0.5f;    ///< Grid cell size [m]
  float cell_percentile = 0.2f;    ///< Percentile for robust minimum (0-1)
  float ground_thickness = 0.3f;   ///< Ground layer thickness [m]
  float max_ground_height = 0.5f;  ///< Max height to consider as ground [m]
  size_t min_points_per_cell = 2;  ///< Min points to process a cell
};

// =============================================================================
// Ground Segmentation Result
// =============================================================================

/**
 * @brief Result of ground segmentation
 */
struct GroundSegResult {
  std::vector<uint32_t> ground;     ///< Indices of ground points
  std::vector<uint32_t> obstacles;  ///< Indices of obstacle points

  /**
   * @brief Check if segmentation produced results
   */
  [[nodiscard]] bool empty() const noexcept {
    return ground.empty() && obstacles.empty();
  }

  /**
   * @brief Get ground point ratio
   */
  [[nodiscard]] double groundRatio() const noexcept {
    size_t total = ground.size() + obstacles.size();
    return total > 0 ? static_cast<double>(ground.size()) / total : 0.0;
  }
};

// =============================================================================
// Ground Segmentation API
// =============================================================================

/**
 * @brief Segment ground points using grid-based height analysis
 *
 * Algorithm:
 * 1. Divide point cloud into 2D grid cells
 * 2. For each cell, compute robust minimum height (using percentile)
 * 3. Classify points as ground if within [cell_min, cell_min + thickness]
 * 4. Points above threshold or in sparse cells are classified as obstacles
 *
 * Advantages over RANSAC:
 * - Works on uneven terrain (slopes, bumps)
 * - O(N) complexity (faster for large clouds)
 * - More robust to outliers via percentile-based minimum
 *
 * @param cloud Input point cloud
 * @param config Segmentation configuration
 * @return GroundSegResult with ground and obstacle indices
 */
[[nodiscard]] GroundSegResult segmentGround(
    const PointCloud& cloud, const GroundSegConfig& config = GroundSegConfig{});

/**
 * @brief Segment ground points (convenience overload)
 *
 * @param cloud Input point cloud
 * @param grid_resolution Grid cell size [m]
 * @param ground_thickness Ground layer thickness [m]
 * @param max_ground_height Maximum height to consider as ground [m]
 * @return GroundSegResult with ground and obstacle indices
 */
[[nodiscard]] GroundSegResult segmentGround(const PointCloud& cloud,
                                            float grid_resolution,
                                            float ground_thickness = 0.3f,
                                            float max_ground_height = 0.5f);

}  // namespace segmentation
}  // namespace npcl

#include "nanopcl/segmentation/impl/ground_seg_impl.hpp"

#endif  // NANOPCL_SEGMENTATION_GROUND_SEG_HPP
