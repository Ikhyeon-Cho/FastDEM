/*
 * voxel_filter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/algorithms/voxel_filter.h"
#include <cmath>
#include <limits>
#include <random>

namespace height_mapping::algorithms {

VoxelFilter::Stats
VoxelFilter::filterInPlace(height_mapping::PointCloud &cloud) const {
  Stats stats;
  stats.original_size = cloud.size();

  if (cloud.empty()) {
    return stats;
  }

  // Store sensor data availability
  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();

  // Group points by voxel
  std::map<VoxelKey, VoxelData> voxel_map;

  for (size_t i = 0; i < cloud.size(); ++i) {
    auto point = cloud[i];

    // Skip invalid points
    if (!point.isFinite()) {
      continue;
    }

    VoxelKey key = getVoxelKey(point.xyz());
    voxel_map[key].points.push_back(point.xyz());
    voxel_map[key].indices.push_back(i);
  }

  stats.voxel_count = voxel_map.size();

  // Create filtered cloud with representative points
  height_mapping::PointCloud filtered;
  filtered.reserve(voxel_map.size());

  for (const auto &[key, voxel_data] : voxel_map) {
    height_mapping::Point3D repr_point =
        computeRepresentativePoint(voxel_data, key);

    // Add point with appropriate sensor data
    if (!has_intensity && !has_color) {
      filtered.push_back(repr_point);
    } else {
      addSensorData(filtered, cloud, voxel_data, repr_point);
    }
  }

  stats.filtered_size = filtered.size();

  // Preserve metadata
  filtered.setTimestamp(cloud.timestamp());
  filtered.setFrameId(cloud.frameId());

  // Swap to avoid copy
  cloud = std::move(filtered);

  return stats;
}

height_mapping::PointCloud
VoxelFilter::filter(const height_mapping::PointCloud &cloud,
                    Stats &stats) const {
  height_mapping::PointCloud result = cloud; // Copy
  stats = filterInPlace(result);             // Modify copy
  return result;
}

VoxelFilter::VoxelKey
VoxelFilter::getVoxelKey(const height_mapping::Point3D &point) const {
  int vx = static_cast<int>(std::floor(point.x / config_.voxel_size));
  int vy = static_cast<int>(std::floor(point.y / config_.voxel_size));
  int vz = static_cast<int>(std::floor(point.z / config_.voxel_size));
  return std::make_tuple(vx, vy, vz);
}

height_mapping::Point3D
VoxelFilter::computeRepresentativePoint(const VoxelData &voxel_data,
                                        const VoxelKey &key) const {

  switch (config_.reduction_method) {
  case ReductionMethod::CENTROID: {
    height_mapping::Point3D centroid(0, 0, 0);
    for (const auto &p : voxel_data.points) {
      centroid += p;
    }
    return centroid * (1.0f / voxel_data.points.size());
  }

  case ReductionMethod::RANDOM: {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, voxel_data.points.size() - 1);
    return voxel_data.points[dis(gen)];
  }

  case ReductionMethod::FIRST: {
    return voxel_data.points.front();
  }

  case ReductionMethod::CLOSEST: {
    // Find point closest to voxel center
    auto [vx, vy, vz] = key;
    height_mapping::Point3D voxel_center((vx + 0.5f) * config_.voxel_size,
                                         (vy + 0.5f) * config_.voxel_size,
                                         (vz + 0.5f) * config_.voxel_size);

    float min_dist_sq = std::numeric_limits<float>::max();
    height_mapping::Point3D closest_point = voxel_data.points.front();

    for (const auto &p : voxel_data.points) {
      float dist_sq = (p - voxel_center).squaredNorm();
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        closest_point = p;
      }
    }
    return closest_point;
  }

  case ReductionMethod::VOXEL_CENTER: {
    auto [vx, vy, vz] = key;
    return height_mapping::Point3D((vx + 0.5f) * config_.voxel_size,
                                   (vy + 0.5f) * config_.voxel_size,
                                   (vz + 0.5f) * config_.voxel_size);
  }

  default:
    return voxel_data.points.front();
  }
}

void VoxelFilter::addSensorData(
    height_mapping::PointCloud &filtered_cloud,
    const height_mapping::PointCloud &original_cloud,
    const VoxelData &voxel_data,
    const height_mapping::Point3D &repr_point) const {

  // For CENTROID and VOXEL_CENTER, average sensor data
  if (config_.reduction_method == ReductionMethod::CENTROID ||
      config_.reduction_method == ReductionMethod::VOXEL_CENTER) {

    float avg_intensity = 0;
    std::array<float, 3> avg_color = {0, 0, 0};

    for (size_t idx : voxel_data.indices) {
      auto point = original_cloud[idx];
      avg_intensity += point.intensity();
      auto color = point.color();
      avg_color[0] += color[0];
      avg_color[1] += color[1];
      avg_color[2] += color[2];
    }

    size_t count = voxel_data.indices.size();
    avg_intensity /= count;

    std::array<uint8_t, 3> final_color = {
        static_cast<uint8_t>(avg_color[0] / count),
        static_cast<uint8_t>(avg_color[1] / count),
        static_cast<uint8_t>(avg_color[2] / count)};

    filtered_cloud.addPoint(repr_point, avg_intensity, final_color);
  }
  // For other methods, use sensor data from selected point
  else {
    size_t selected_idx = voxel_data.indices.front();

    if (config_.reduction_method == ReductionMethod::RANDOM) {
      static std::random_device rd;
      static std::mt19937 gen(rd());
      std::uniform_int_distribution<size_t> dis(0,
                                                voxel_data.indices.size() - 1);
      selected_idx = voxel_data.indices[dis(gen)];
    } else if (config_.reduction_method == ReductionMethod::CLOSEST) {
      // Find which original point is closest to representative
      float min_dist_sq = std::numeric_limits<float>::max();
      for (size_t idx : voxel_data.indices) {
        float dist_sq =
            (voxel_data.points[idx - voxel_data.indices.front()] - repr_point)
                .squaredNorm();
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          selected_idx = idx;
        }
      }
    }

    auto selected_point = original_cloud[selected_idx];
    filtered_cloud.addPoint(repr_point, selected_point.intensity(),
                            selected_point.color());
  }
}

} // namespace height_mapping::algorithms