/*
 * passthrough_filter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/algorithms/passthrough_filter.h"
#include <cmath>

namespace height_mapping::algorithms {

PassthroughFilter::Stats
PassthroughFilter::filterInPlace(height_mapping::PointCloud &cloud) const {
  Stats stats;
  stats.original_size = cloud.size();

  if (cloud.empty()) {
    return stats;
  }

  // Filter points using in-place algorithm
  height_mapping::PointCloud filtered;
  filtered.reserve(cloud.size());

  // Store sensor data availability
  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();

  // Filter points
  for (size_t i = 0; i < cloud.size(); ++i) {
    auto point = cloud[i];

    if (passesFilter(point)) {
      // Add point with appropriate sensor data
      if (!has_intensity && !has_color) {
        filtered.push_back(point.xyz());
      } else if (has_intensity && !has_color) {
        filtered.addPoint(point.xyz(), point.intensity());
      } else if (!has_intensity && has_color) {
        filtered.addPoint(point.xyz(), 0.0f, point.color());
      } else {
        filtered.addPoint(point.xyz(), point.intensity(), point.color());
      }
    }
  }

  stats.filtered_size = filtered.size();
  stats.removed_count = stats.original_size - stats.filtered_size;

  // Preserve metadata
  filtered.setTimestamp(cloud.timestamp());
  filtered.setFrameId(cloud.frameId());

  // Swap to avoid copy
  cloud = std::move(filtered);

  return stats;
}

height_mapping::PointCloud
PassthroughFilter::filter(const height_mapping::PointCloud &cloud,
                          Stats &stats) const {
  height_mapping::PointCloud result = cloud; // Copy
  stats = filterInPlace(result);             // Modify copy
  return result;
}

bool PassthroughFilter::passesFilter(
    height_mapping::PointCloud::PointView point) const {
  // Check if point is finite
  if (!point.isFinite()) {
    return false;
  }

  // Check axis-aligned bounds
  if (point.x() < config_.x_min || point.x() > config_.x_max ||
      point.y() < config_.y_min || point.y() > config_.y_max ||
      point.z() < config_.z_min || point.z() > config_.z_max) {
    return false;
  }

  // Check distance filter if enabled
  if (config_.use_distance_filter) {
    float distance = point.norm();
    if (distance < config_.distance_min || distance > config_.distance_max) {
      return false;
    }
  }

  return true;
}

} // namespace height_mapping::algorithms