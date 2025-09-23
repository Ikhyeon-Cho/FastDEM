/*
 * raycasting.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/raycasting.h"
#include "height_mapping_core/interfaces/transform_provider.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <cmath>
#include <iostream>

namespace height_mapping::core::stages {

Raycasting::Raycasting() : Stage("Raycasting") {}

void Raycasting::configure(const std::map<std::string, std::string> &params) {
  auto it = params.find("max_ground_angle");
  if (it != params.end()) {
    max_ground_angle_ =
        std::stof(it->second) * M_PI / 180.0f; // Convert to radians
  }

  it = params.find("correction_threshold");
  if (it != params.end()) {
    correction_threshold_ = std::stof(it->second);
  }

  it = params.find("enable_correction");
  if (it != params.end()) {
    enable_correction_ = (it->second == "true" || it->second == "1");
  }
}

void Raycasting::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  auto &map = mapping_ctx.map();
  auto &cloud = mapping_ctx.cloud();

  if (cloud.empty() || !enable_correction_) {
    return;
  }

  // Get sensor origin from transform provider
  Eigen::Vector3f sensor_origin(0, 0, 0);
  auto transform_provider = ctx.getService<ITransformProvider>();
  if (transform_provider) {
    // Get transform from map to sensor frame to find sensor position in map
    auto transform_opt = transform_provider->lookupTransform(
        cloud.frameId(), "base_link", cloud.timestamp());
    if (transform_opt.has_value()) {
      // Sensor is typically at base_link origin
      sensor_origin = transform_opt.value().translation();
    }
  }

  // Get map layers
  namespace layer = height_map::layer;
  auto &elevation = map[layer::elevation];
  auto &variance = map[layer::variance];

  // Create or get min ray height layer for this scan
  if (!map.exists("min_ray_height")) {
    map.add("min_ray_height");
  }
  auto &min_ray_height = map["min_ray_height"];
  min_ray_height.setConstant(INFINITY);

  // Process each point
  size_t ground_points = 0;
  size_t corrected_cells = 0;

  for (const auto point : cloud) {  // PointView from iterator
    // Skip invalid points
    if (!point.isFinite()) {
      continue;
    }

    // Calculate ray angle (negative = pointing down)
    float dx = point.x() - sensor_origin.x();
    float dy = point.y() - sensor_origin.y();
    float dz = point.z() - sensor_origin.z();
    float horizontal_dist = std::sqrt(dx * dx + dy * dy);

    if (horizontal_dist < 0.1f)
      continue; // Skip near-vertical rays

    float ray_angle = std::atan2(dz, horizontal_dist);

    // Only process rays that could hit ground (pointing downward)
    bool is_ground_ray = (ray_angle < max_ground_angle_);
    if (is_ground_ray) {
      ground_points++;
    }

    // Trace ray and track minimum height
    float ray_length = std::sqrt(dx * dx + dy * dy + dz * dz);
    Eigen::Vector3f ray_dir(dx / ray_length, dy / ray_length, dz / ray_length);

    // Step along ray
    float step = map.getResolution();
    for (float t = 0; t < ray_length - step; t += step) {
      Eigen::Vector3f ray_point = sensor_origin + ray_dir * t;

      // Get cell index
      grid_map::Position pos(ray_point.x(), ray_point.y());
      grid_map::Index idx;
      if (!map.getIndex(pos, idx))
        continue;

      // Track minimum ray height at this cell
      float &min_height = min_ray_height(idx(0), idx(1));
      min_height = std::min(min_height, ray_point.z());
    }
  }

  // Second pass: Apply height corrections using minimum ray heights
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const auto i = (*it)(0);
    const auto j = (*it)(1);

    float &elev = elevation(i, j);
    float &var = variance(i, j);
    float min_ray = min_ray_height(i, j);

    // Only correct if we have ray data and current elevation is too high
    if (std::isfinite(min_ray) && std::isfinite(elev)) {
      if (elev > min_ray + correction_threshold_) {
        // Ground cannot be higher than ray path
        elev = min_ray + correction_threshold_;
        var *= 1.5f; // Increase uncertainty for corrected cells
        corrected_cells++;
      }
    }
  }

  // Update statistics
  total_rays_cast_ += cloud.size();
  cells_corrected_ += corrected_cells;
  ground_points_detected_ += ground_points;
}

// Register this stage with the factory
REGISTER_STAGE(Raycasting)

} // namespace height_mapping::core::stages