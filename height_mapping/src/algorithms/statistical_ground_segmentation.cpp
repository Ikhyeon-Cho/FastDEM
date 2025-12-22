/*
 * statistical_ground_segmentation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/algorithms/algorithms.h"

#include <algorithm>
#include <vector>

namespace height_mapping::algorithms {

namespace {

float calculateRobustMinimum(const std::vector<float>& z_values,
                             float percentile) {
  size_t idx = static_cast<size_t>(percentile * z_values.size());
  idx = std::max(size_t(0), std::min(idx, z_values.size() - 1));
  return z_values[idx];
}

PointCloud classifyAndFilter(const PointCloud& cloud, float robust_min_z,
                             float ground_threshold,
                             const StatisticalGroundSegmentationConfig& config,
                             GroundSegmentationStats& stats) {
  PointCloud filtered;
  filtered.reserve(cloud.size());

  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();
  if (has_intensity) filtered.enableIntensity();
  if (has_color) filtered.enableColor();

  auto addPointWithData = [&](size_t idx) {
    filtered.points.push_back(cloud.points[idx]);
    if (has_intensity) filtered.intensity().push_back(cloud.intensity()[idx]);
    if (has_color) filtered.color().push_back(cloud.color()[idx]);
  };

  for (size_t i = 0; i < cloud.size(); ++i) {
    float z = cloud.points[i].z();

    if (z < robust_min_z - config.noise_threshold) {
      stats.noise_count++;
    } else if (z <= ground_threshold) {
      stats.ground_count++;
      if (config.keep_only_ground) {
        addPointWithData(i);
      }
    } else {
      stats.obstacle_count++;
      if (!config.keep_only_ground) {
        addPointWithData(i);
      }
    }
  }

  return filtered;
}

}  // namespace

GroundSegmentationStats segmentGroundStatistical(
    PointCloud& cloud, const StatisticalGroundSegmentationConfig& config) {
  GroundSegmentationStats stats;

  if (cloud.empty()) {
    return stats;
  }

  // Collect and sort z values
  std::vector<float> z_values;
  z_values.reserve(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    z_values.push_back(cloud.points[i].z());
  }
  std::sort(z_values.begin(), z_values.end());

  // Calculate thresholds
  stats.robust_min_z = calculateRobustMinimum(z_values, config.ground_percentile);
  stats.ground_threshold = stats.robust_min_z + config.ground_thickness;

  // Classify and filter
  auto filtered = classifyAndFilter(cloud, stats.robust_min_z,
                                    stats.ground_threshold, config, stats);

  // Preserve metadata and swap
  filtered.setTimestamp(cloud.timestamp());
  filtered.setFrameId(cloud.frameId());
  cloud = std::move(filtered);

  return stats;
}

}  // namespace height_mapping::algorithms
