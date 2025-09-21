/*
 * raycasting.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/raycasting.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <iostream>
#include <cmath>

namespace height_mapping::core {

RaycastingStage::RaycastingStage()
    : Stage("Raycasting", "3D to 2.5D Projection") {
}

void RaycastingStage::configure(const std::map<std::string, std::string>& params) {
  auto it = params.find("ray_length_max");
  if (it != params.end()) {
    ray_length_max_ = std::stof(it->second);
  }

  it = params.find("ray_length_min");
  if (it != params.end()) {
    ray_length_min_ = std::stof(it->second);
  }

  it = params.find("enable_clearing");
  if (it != params.end()) {
    enable_clearing_ = (it->second == "true" || it->second == "1");
  }

  it = params.find("clearing_height_threshold");
  if (it != params.end()) {
    clearing_height_threshold_ = std::stof(it->second);
  }
}

void RaycastingStage::processImpl(pipeline::Context& ctx) {
  auto& mapping_ctx = static_cast<MappingContext&>(ctx);

  auto& map = mapping_ctx.map();
  if (map.empty()) {
    std::cerr << "[Raycasting] No height map in context" << std::endl;
    return;
  }

  auto& cloud = mapping_ctx.cloud();
  if (cloud.empty()) {
    return;
  }

  // Get sensor origin (assuming it's at 0,0,0 in cloud frame)
  // In real implementation, this should come from transform
  float sensor_x = 0.0f;
  float sensor_y = 0.0f;
  float sensor_z = 0.0f;

  size_t rays_cast = 0;
  size_t cells_updated = 0;

  // Process each point
  for (const auto& point : cloud.points) {
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    // Calculate ray length
    float dx = point.x - sensor_x;
    float dy = point.y - sensor_y;
    float dz = point.z - sensor_z;
    float ray_length = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Check ray length bounds
    if (ray_length < ray_length_min_ || ray_length > ray_length_max_) {
      continue;
    }

    rays_cast++;

    // Here we would implement actual raycasting algorithm:
    // 1. Trace ray from sensor to point
    // 2. Clear cells along the ray if enable_clearing_
    // 3. Update height at endpoint
    // This is a simplified placeholder

    // Update statistics
    cells_updated++;
    if (enable_clearing_) {
      // Would count actual cleared cells
      cells_cleared_++;
    }
  }

  // Update total statistics
  total_rays_cast_ += rays_cast;
  this->cells_updated_ += cells_updated;

  // Store in context
}

// Register this stage with the factory
REGISTER_STAGE(RaycastingStage)

} // namespace height_mapping::core