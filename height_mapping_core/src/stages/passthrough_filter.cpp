/*
 * passthrough_filter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/passthrough_filter.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <cmath>
#include <iostream>

namespace height_mapping::core {

PassthroughFilterStage::PassthroughFilterStage()
    : Stage("PassthroughFilter", "Point Cloud Filter") {
}

void PassthroughFilterStage::configure(const std::map<std::string, std::string>& params) {
  // X limits
  auto it = params.find("x_min");
  if (it != params.end()) {
    x_min_ = std::stof(it->second);
  }
  it = params.find("x_max");
  if (it != params.end()) {
    x_max_ = std::stof(it->second);
  }

  // Y limits
  it = params.find("y_min");
  if (it != params.end()) {
    y_min_ = std::stof(it->second);
  }
  it = params.find("y_max");
  if (it != params.end()) {
    y_max_ = std::stof(it->second);
  }

  // Z limits
  it = params.find("z_min");
  if (it != params.end()) {
    z_min_ = std::stof(it->second);
  }
  it = params.find("z_max");
  if (it != params.end()) {
    z_max_ = std::stof(it->second);
  }

  // Distance limits
  it = params.find("distance_min");
  if (it != params.end()) {
    distance_min_ = std::stof(it->second);
    use_distance_filter_ = true;
  }
  it = params.find("distance_max");
  if (it != params.end()) {
    distance_max_ = std::stof(it->second);
    use_distance_filter_ = true;
  }
}

void PassthroughFilterStage::processImpl(pipeline::Context& ctx) {
  auto& mapping_ctx = static_cast<MappingContext&>(ctx);

  auto& cloud = mapping_ctx.cloud();
  if (cloud.empty()) {
    return;
  }

  size_t points_before = cloud.size();

  // Create filtered cloud
  height_map::PointCloudXYZ filtered_cloud;
  filtered_cloud.reserve(cloud.size());
  filtered_cloud.frame_id = cloud.frame_id;
  filtered_cloud.timestamp = cloud.timestamp;

  // Apply filters point by point
  for (const auto& point : cloud.points) {
    // Check NaN
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    // Check axis-aligned bounds
    if (point.x < x_min_ || point.x > x_max_ ||
        point.y < y_min_ || point.y > y_max_ ||
        point.z < z_min_ || point.z > z_max_) {
      continue;
    }

    // Check distance filter if enabled
    if (use_distance_filter_) {
      float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (distance < distance_min_ || distance > distance_max_) {
        continue;
      }
    }

    // Point passed all filters
    filtered_cloud.push_back(point);
  }

  // Update cloud
  cloud = std::move(filtered_cloud);

  // Statistics could be logged if needed
  // size_t points_after = cloud.size();
}

// Register this stage with the factory
REGISTER_STAGE(PassthroughFilterStage)

} // namespace height_mapping::core