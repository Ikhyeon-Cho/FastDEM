/*
 * transform_cloud.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/transform_cloud.h"
#include "height_mapping_core/interfaces/transform_provider.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <iostream>
#include <stdexcept>

namespace height_mapping::core::stages {

TransformCloud::TransformCloud() : Stage("TransformCloud") {}

void TransformCloud::configure(
    const std::map<std::string, std::string> &params) {
  // Configure target frame
  auto it = params.find("target_frame");
  if (it != params.end()) {
    target_frame_ = it->second;
  }
}

void TransformCloud::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  auto &cloud = mapping_ctx.cloud();

  // Skip if cloud is empty or already in target frame
  if (cloud.empty() || cloud.frameId() == target_frame_) {
    return;
  }

  // Get transform provider from context (required)
  auto transform_provider = ctx.getService<ITransformProvider>();
  if (!transform_provider) {
    throw std::runtime_error("TransformCloud stage requires a transform "
                             "provider to be set in context");
  }

  // Lookup transform
  auto transform_opt = transform_provider->lookupTransform(
      target_frame_, cloud.frameId(), cloud.timestamp());

  if (!transform_opt.has_value()) {
    throw std::runtime_error("Failed to get transform from '" + cloud.frameId() +
                             "' to '" + target_frame_ + "'");
  }

  // Apply transform to all points
  const auto &T = transform_opt.value().matrix();
  for (auto point : cloud) {  // PointView from iterator
    float x = point.x();
    float y = point.y();
    float z = point.z();

    point.x() = T(0, 0) * x + T(0, 1) * y + T(0, 2) * z + T(0, 3);
    point.y() = T(1, 0) * x + T(1, 1) * y + T(1, 2) * z + T(1, 3);
    point.z() = T(2, 0) * x + T(2, 1) * y + T(2, 2) * z + T(2, 3);
  }

  // Update frame_id after successful transform
  cloud.setFrameId(target_frame_);
}

// Register this stage with the factory
REGISTER_STAGE(TransformCloud)

} // namespace height_mapping::core::stages