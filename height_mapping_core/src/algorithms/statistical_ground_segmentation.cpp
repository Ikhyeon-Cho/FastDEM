/*
 * statistical_ground_segmentation.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/algorithms/statistical_ground_segmentation.h"
#include <algorithm>

namespace height_mapping::algorithms {

StatisticalGroundSegmentation::Stats
StatisticalGroundSegmentation::segmentInPlace(
    height_mapping::PointCloud &cloud) const {
  Stats stats;

  if (cloud.empty()) {
    return stats;
  }

  // Collect all z values for percentile calculation
  std::vector<float> all_z_values;
  all_z_values.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    all_z_values.push_back(cloud[i].z());
  }

  // Sort for percentile calculation
  std::sort(all_z_values.begin(), all_z_values.end());

  // Calculate robust minimum using percentile
  stats.robust_min_z = calculateRobustMinimum(all_z_values);
  stats.ground_threshold = stats.robust_min_z + config_.ground_thickness;

  // Classify and filter points
  Stats classification_stats;
  auto filtered = classifyAndFilter(
      cloud, stats.robust_min_z, stats.ground_threshold, classification_stats);

  stats.ground_count = classification_stats.ground_count;
  stats.obstacle_count = classification_stats.obstacle_count;
  stats.noise_count = classification_stats.noise_count;

  // Preserve metadata
  filtered.setTimestamp(cloud.timestamp());
  filtered.setFrameId(cloud.frameId());

  // Swap to avoid copy
  cloud = std::move(filtered);

  return stats;
}

height_mapping::PointCloud
StatisticalGroundSegmentation::segment(const height_mapping::PointCloud &cloud,
                                       Stats &stats) const {
  height_mapping::PointCloud result = cloud; // Copy
  stats = segmentInPlace(result);            // Modify copy
  return result;
}

float StatisticalGroundSegmentation::calculateRobustMinimum(
    const std::vector<float> &z_values) const {

  // Use configured percentile as robust minimum (ignores outlier noise points)
  // This filters out deep noise spikes that occasionally appear below ground
  size_t percentile_idx =
      static_cast<size_t>(config_.ground_percentile * z_values.size());
  percentile_idx =
      std::max(size_t(0), std::min(percentile_idx, z_values.size() - 1));

  return z_values[percentile_idx];
}

height_mapping::PointCloud StatisticalGroundSegmentation::classifyAndFilter(
    const height_mapping::PointCloud &cloud, float robust_min_z,
    float ground_threshold, Stats &stats) const {

  height_mapping::PointCloud filtered;
  filtered.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    auto point = cloud[i];
    float z = point.z();

    if (z < robust_min_z - config_.noise_threshold) {
      // Deep outlier noise - ignore completely
      stats.noise_count++;
    } else if (z <= ground_threshold) {
      // Ground point
      stats.ground_count++;
      if (config_.keep_only_ground) {
        filtered.addPoint(point.xyz(), point.intensity(), point.color());
      }
    } else {
      // Obstacle point
      stats.obstacle_count++;
      if (!config_.keep_only_ground) {
        filtered.addPoint(point.xyz(), point.intensity(), point.color());
      }
    }
  }

  return filtered;
}

} // namespace height_mapping::algorithms