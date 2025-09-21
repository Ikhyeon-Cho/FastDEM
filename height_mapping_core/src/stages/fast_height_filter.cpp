/*
 * fast_height_filter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/fast_height_filter.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace height_mapping::core {

FastHeightFilterStage::FastHeightFilterStage()
    : Stage("FastHeightFilter", "Height-based Filter") {
}

void FastHeightFilterStage::configure(const std::map<std::string, std::string>& params) {
  auto it = params.find("min_height");
  if (it != params.end()) {
    min_height_ = std::stof(it->second);
  }

  it = params.find("max_height");
  if (it != params.end()) {
    max_height_ = std::stof(it->second);
  }

  it = params.find("reference_height");
  if (it != params.end()) {
    reference_height_ = std::stof(it->second);
  }

  it = params.find("adaptive");
  if (it != params.end()) {
    adaptive_filtering_ = (it->second == "true" || it->second == "1");
  }
}

void FastHeightFilterStage::processImpl(pipeline::Context& ctx) {
  auto& mapping_ctx = static_cast<MappingContext&>(ctx);

  auto& cloud = mapping_ctx.cloud();
  if (cloud.empty()) {
    return;
  }

  size_t points_before = cloud.size();

  // If adaptive filtering is enabled, compute reference height from data
  float dynamic_reference = reference_height_;
  if (adaptive_filtering_ && !cloud.empty()) {
    // Compute median or mean Z value as reference
    std::vector<float> z_values;
    z_values.reserve(cloud.size());

    for (const auto& point : cloud.points) {
      if (std::isfinite(point.z)) {
        z_values.push_back(point.z);
      }
    }

    if (!z_values.empty()) {
      // Use median as reference (more robust to outliers)
      std::nth_element(z_values.begin(), z_values.begin() + z_values.size()/2, z_values.end());
      dynamic_reference = z_values[z_values.size()/2];
    }
  }

  // Filter points
  height_map::PointCloudXYZ filtered_cloud;
  filtered_cloud.reserve(cloud.size());
  filtered_cloud.frame_id = cloud.frame_id;
  filtered_cloud.timestamp = cloud.timestamp;

  float height_min = dynamic_reference + min_height_;
  float height_max = dynamic_reference + max_height_;

  for (const auto& point : cloud.points) {
    // Check if point is within height bounds
    if (point.z >= height_min && point.z <= height_max) {
      filtered_cloud.push_back(point);
    }
  }

  // Update cloud
  cloud = std::move(filtered_cloud);

  size_t points_after = cloud.size();
  size_t points_removed = points_before - points_after;
  total_points_filtered_ += points_removed;

  // Store statistics in context
}

// Register this stage with the factory
REGISTER_STAGE(FastHeightFilterStage)

} // namespace height_mapping::core