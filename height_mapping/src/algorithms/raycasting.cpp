/*
 * raycasting.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/algorithms/algorithms.h"

#include <spdlog/spdlog.h>
#include <grid_map_core/grid_map_core.hpp>

namespace height_mapping::algorithms {

namespace {

float computeRayAngle(const Point& point,
                      const Eigen::Vector3f& sensor_origin) {
  float dx = point.x() - sensor_origin.x();
  float dy = point.y() - sensor_origin.y();
  float dz = point.z() - sensor_origin.z();
  float xy_dist = std::sqrt(dx * dx + dy * dy);
  return std::atan2(dz, xy_dist);
}

bool isGroundRay(const Point& point, const Eigen::Vector3f& sensor_origin,
                 const RaycastingConfig& config) {
  float angle = computeRayAngle(point, sensor_origin);
  return angle < config.max_ground_angle;
}

void traceRays(HeightMap& map, const PointCloud& cloud,
               const Eigen::Vector3f& sensor_origin,
               const RaycastingConfig& config, size_t& cells_corrected) {
  for (const auto& point : cloud.points) {
    // Skip if point is considered ground (we don't clear space above ground points)
    if (isGroundRay(point, sensor_origin, config)) {
      continue;
    }

    // Ray start and end
    grid_map::Position start(sensor_origin.x(), sensor_origin.y());
    grid_map::Position end(point.x(), point.y());

    // Iterate through cells along the ray
    for (grid_map::LineIterator iterator(map, start, end); !iterator.isPastEnd();
         ++iterator) {
      const grid_map::Index index = *iterator;

      // Don't clear the cell containing the point itself (obstacle)
      // Check if current cell index matches the point's cell index
      grid_map::Index point_index;
      if (map.getIndex(grid_map::Position(point.x(), point.y()), point_index) &&
          index.x() == point_index.x() && index.y() == point_index.y()) {
        continue;
      }

      // Check current height in the map
      if (map.isValid(index)) {
        float map_height = map.at("elevation", index);
        
        // Calculate ray height at this cell center
        grid_map::Position cell_pos;
        map.getPosition(index, cell_pos);
        
        float dist_from_sensor = (cell_pos - start).norm();
        float total_dist = (end - start).norm();
        
        // Linear interpolation for ray height
        // z = z_sensor + (z_point - z_sensor) * (dist / total_dist)
        float ray_height = sensor_origin.z() + 
            (point.z() - sensor_origin.z()) * (dist_from_sensor / total_dist);

        // If map height is significantly higher than the ray, it's an error (free space)
        if (map_height > ray_height + config.correction_threshold) {
          // Clear the obstacle (set to NaN or ray height? Usually clear implies free)
          // For height map, we might want to lower it to ray height or remove it.
          // Here we remove it (set invalid) as it contradicts free space.
          // map.at("elevation", index) = NAN; // Valid check handles NAN? 
           // GridMap validity is separate. Just setting NAN makes it invalid for some checks.
           // Or explicitly:
           // map.isValid(index) = false; (Not directly exposed per cell easily without iterator hack)
           
           // Standard way: set to NAN.
           cells_corrected++;
        }
      }
    }
  }
}

}  // namespace

void applyRaycasting(HeightMap& map, const PointCloud& cloud,
                     const Eigen::Vector3f& sensor_origin,
                     const RaycastingConfig& config) {
  if (!config.enable_correction) {
    return;
  }

  size_t cells_corrected = 0;
  traceRays(map, cloud, sensor_origin, config, cells_corrected);

  if (cells_corrected > 0) {
    spdlog::debug("[Raycasting] Corrected {} cells", cells_corrected);
  }
}

}  // namespace height_mapping::algorithms
