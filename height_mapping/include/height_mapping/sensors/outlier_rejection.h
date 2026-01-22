/*
 * outlier_rejection.h
 *
 * Reject outlier points based on sensor uncertainty and existing map data.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_SENSORS_OUTLIER_REJECTION_H
#define HEIGHT_MAPPING_SENSORS_OUTLIER_REJECTION_H

#include <nanopcl/core.hpp>
#include <vector>
#include <cmath>

#include "height_mapping/config/outlier_rejection.h"
#include "height_mapping/core/layers.h"
#include "height_mapping/core/map.h"

namespace height_mapping::sensors {

/**
 * @brief Reject outlier points based on k-sigma test against existing map.
 *
 * Points where |z - existing_elevation| > k × σ are rejected.
 * Points outside map bounds or in uninitialized cells are kept.
 *
 * @param map Height map with existing elevation data
 * @param config Outlier rejection configuration
 * @param cloud Point cloud (modified in-place)
 * @param uncertainties Per-point uncertainties σ (modified in-place)
 *
 * @code
 *   config::OutlierRejection cfg{.enabled = true, .sigma_threshold = 3.0f};
 *   sensors::rejectOutliers(map, cfg, cloud, uncertainties);
 * @endcode
 */
inline void rejectOutliers(const HeightMap& map,
                           const config::OutlierRejection& config,
                           npcl::PointCloud& cloud,
                           std::vector<float>& uncertainties) {
  if (!config.enabled) return;
  if (!map.exists(layer::mean)) return;  // No existing data to compare

  const float k = config.sigma_threshold;
  const float min_sigma = config.min_uncertainty;
  const auto& mean_mat = map.get(layer::mean);

  // Build list of indices to keep
  std::vector<size_t> keep_indices;
  keep_indices.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];  // Point is Eigen::Vector3f
    grid_map::Index idx;

    // Check if point is within map bounds
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), idx)) {
      keep_indices.push_back(i);  // Keep points outside map bounds
      continue;
    }

    const float existing_elevation = mean_mat(idx(0), idx(1));
    if (!std::isfinite(existing_elevation)) {
      keep_indices.push_back(i);  // Keep points for uninitialized cells
      continue;
    }

    // Apply k-sigma test with minimum uncertainty floor
    const float sigma = std::max(uncertainties[i], min_sigma);
    const float diff = std::abs(point.z() - existing_elevation);
    if (diff <= k * sigma) {
      keep_indices.push_back(i);  // Keep inlier points
    }
  }

  // Skip if no points were rejected
  if (keep_indices.size() == cloud.size()) return;

  // Use PointCloud's index operator to extract subset (preserves all attributes)
  npcl::PointCloud filtered_cloud = cloud[keep_indices];

  // Build filtered uncertainties
  std::vector<float> filtered_uncertainties;
  filtered_uncertainties.reserve(keep_indices.size());
  for (size_t idx : keep_indices) {
    filtered_uncertainties.push_back(uncertainties[idx]);
  }

  cloud = std::move(filtered_cloud);
  uncertainties = std::move(filtered_uncertainties);
}

}  // namespace height_mapping::sensors

#endif  // HEIGHT_MAPPING_SENSORS_OUTLIER_REJECTION_H
