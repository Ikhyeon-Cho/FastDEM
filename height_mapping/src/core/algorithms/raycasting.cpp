/*
 * raycasting.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/core/algorithms/raycasting.h"

#include <cmath>
#include <limits>

namespace height_mapping::algorithms {

Raycasting::Stats Raycasting::correctMap(
    height_mapping::HeightMap &map, const height_mapping::PointCloud &cloud,
    const Eigen::Vector3f &sensor_origin) const {
  Stats stats;

  if (cloud.empty() || !config_.enable_correction) {
    return stats;
  }

  // Create or get min ray height layer for this scan
  if (!map.exists("min_ray_height")) {
    map.add("min_ray_height");
  }
  auto &min_ray_height = map["min_ray_height"];
  min_ray_height.setConstant(std::numeric_limits<float>::infinity());

  // First pass: trace rays and compute minimum ray heights
  traceRays(map, cloud, sensor_origin, stats);

  // Second pass: apply height corrections
  applyCorrections(map, stats);

  // Update cumulative statistics
  cumulative_stats_.total_rays_cast += stats.total_rays_cast;
  cumulative_stats_.ground_points_detected += stats.ground_points_detected;
  cumulative_stats_.cells_corrected += stats.cells_corrected;
  cumulative_stats_.cells_traced += stats.cells_traced;

  return stats;
}

void Raycasting::traceRays(height_mapping::HeightMap &map,
                           const height_mapping::PointCloud &cloud,
                           const Eigen::Vector3f &sensor_origin,
                           Stats &stats) const {
  auto &min_ray_height = map["min_ray_height"];

  // Determine step size
  float step =
      (config_.ray_step_size > 0) ? config_.ray_step_size : map.getResolution();

  for (const auto point : cloud) {
    // Skip invalid points
    if (!point.isFinite()) {
      continue;
    }

    stats.total_rays_cast++;

    // Check if this is a ground ray
    if (isGroundRay(point, sensor_origin)) {
      stats.ground_points_detected++;
    }

    // Calculate ray direction and length
    float dx = point.x() - sensor_origin.x();
    float dy = point.y() - sensor_origin.y();
    float dz = point.z() - sensor_origin.z();
    float ray_length = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (ray_length < 0.01f) continue;  // Skip very short rays

    Eigen::Vector3f ray_dir(dx / ray_length, dy / ray_length, dz / ray_length);

    // Step along ray
    for (float t = 0; t < ray_length - step; t += step) {
      Eigen::Vector3f ray_point = sensor_origin + ray_dir * t;

      // Get cell index
      grid_map::Position pos(ray_point.x(), ray_point.y());
      grid_map::Index idx;
      if (!map.getIndex(pos, idx)) {
        continue;
      }

      // Track minimum ray height at this cell
      float &min_height = min_ray_height(idx(0), idx(1));
      min_height = std::min(min_height, ray_point.z());
      stats.cells_traced++;
    }
  }
}

void Raycasting::applyCorrections(height_mapping::HeightMap &map,
                                  Stats &stats) const {
  namespace layer = height_mapping::layer;
  auto &elevation = map[layer::elevation];
  auto &variance = map[layer::variance];
  auto &min_ray_height = map["min_ray_height"];

  // Apply corrections to cells where elevation is higher than ray path
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const auto i = (*it)(0);
    const auto j = (*it)(1);

    float &elev = elevation(i, j);
    float &var = variance(i, j);
    float min_ray = min_ray_height(i, j);

    // Only correct if we have ray data and current elevation is too high
    if (std::isfinite(min_ray) && std::isfinite(elev)) {
      if (elev > min_ray + config_.correction_threshold) {
        // Ground cannot be higher than ray path
        elev = min_ray + config_.correction_threshold;
        var *= 1.5f;  // Increase uncertainty for corrected cells
        stats.cells_corrected++;
      }
    }
  }
}

bool Raycasting::isGroundRay(
    const height_mapping::PointCloud::ConstPointView &point,
    const Eigen::Vector3f &sensor_origin) const {
  float angle = computeRayAngle(point, sensor_origin);
  return (angle < config_.max_ground_angle);
}

float Raycasting::computeRayAngle(
    const height_mapping::PointCloud::ConstPointView &point,
    const Eigen::Vector3f &sensor_origin) const {
  float dx = point.x() - sensor_origin.x();
  float dy = point.y() - sensor_origin.y();
  float dz = point.z() - sensor_origin.z();
  float horizontal_dist = std::sqrt(dx * dx + dy * dy);

  if (horizontal_dist < 0.1f) {
    return M_PI / 2;  // Near-vertical ray
  }

  return std::atan2(dz, horizontal_dist);
}

}  // namespace height_mapping::algorithms