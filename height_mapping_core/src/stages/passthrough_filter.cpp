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

namespace height_mapping::core::stages {

using PointCloud = geometry::PointCloud;

PassthroughFilter::PassthroughFilter() : Stage("PassthroughFilter") {}

void PassthroughFilter::configure(
    const std::map<std::string, std::string> &params) {
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

void PassthroughFilter::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  auto &cloud = mapping_ctx.cloud();

  if (cloud.empty()) {
    return;
  }

  // Use the new smart filterInPlace with automatic sensor data handling
  cloud.filterInPlace([this](const auto& point) {
    // point is a ConstPointView with unified access to all data

    // Check if point is finite
    if (!point.isFinite()) {
      return false;
    }

    // Check axis-aligned bounds
    if (point.x() < x_min_ || point.x() > x_max_ ||
        point.y() < y_min_ || point.y() > y_max_ ||
        point.z() < z_min_ || point.z() > z_max_) {
      return false;
    }

    // Check distance filter if enabled
    if (use_distance_filter_) {
      float distance = point.norm();
      if (distance < distance_min_ || distance > distance_max_) {
        return false;
      }
    }

    // Point passed all filters
    return true;
  });
}

// Register this stage with the factory
REGISTER_STAGE(PassthroughFilter)

} // namespace height_mapping::core::stages