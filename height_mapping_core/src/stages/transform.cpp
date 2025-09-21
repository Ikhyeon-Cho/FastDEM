/*
 * transform.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/transform.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <sstream>
#include <iostream>

namespace height_mapping::core {

TransformStage::TransformStage()
    : Stage("Transform", "Coordinate Transform") {
}

void TransformStage::configure(const std::map<std::string, std::string>& params) {
  // Configure target frame
  auto it = params.find("target_frame");
  if (it != params.end()) {
    target_frame_ = it->second;
  }

  // Configure static transform if provided
  it = params.find("static_transform");
  if (it != params.end()) {
    try {
      static_transform_ = parseTransformMatrix(it->second);
      use_static_transform_ = true;
    } catch (const std::exception& e) {
      std::cerr << "[Transform] Failed to parse transform matrix: " << e.what() << std::endl;
    }
  }
}

void TransformStage::processImpl(pipeline::Context& ctx) {
  auto& mapping_ctx = static_cast<MappingContext&>(ctx);

  auto& cloud = mapping_ctx.cloud();
  if (cloud.empty()) {
    return;
  }

  // Apply transformation
  if (use_static_transform_) {
    // Custom transform implementation
    for (auto& point : cloud.points) {
      // Transform point: p' = R * p + t
      // Where the 4x4 matrix is:
      // [ R11 R12 R13 tx ]
      // [ R21 R22 R23 ty ]
      // [ R31 R32 R33 tz ]
      // [  0   0   0   1 ]
      float x = point.x;
      float y = point.y;
      float z = point.z;

      point.x = static_transform_(0, 0) * x + static_transform_(0, 1) * y +
                static_transform_(0, 2) * z + static_transform_(0, 3);
      point.y = static_transform_(1, 0) * x + static_transform_(1, 1) * y +
                static_transform_(1, 2) * z + static_transform_(1, 3);
      point.z = static_transform_(2, 0) * x + static_transform_(2, 1) * y +
                static_transform_(2, 2) * z + static_transform_(2, 3);
    }

    // Update frame_id in cloud
    cloud.frame_id = target_frame_;
  } else {
    // Dynamic transform would require ITransformProvider
    // For now, just update frame_id if different
    if (cloud.frame_id != target_frame_) {
      std::cerr << "[Transform] Warning: Dynamic transform not implemented, "
                << "only updating frame_id from " << cloud.frame_id
                << " to " << target_frame_ << std::endl;
      cloud.frame_id = target_frame_;
    }
  }
}

Eigen::Matrix4f TransformStage::parseTransformMatrix(const std::string& str) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  std::stringstream ss(str);
  std::string value;
  int idx = 0;

  // Parse 16 values for 4x4 matrix
  while (std::getline(ss, value, ',') && idx < 16) {
    try {
      float val = std::stof(value);
      transform(idx / 4, idx % 4) = val;
      idx++;
    } catch (const std::exception& e) {
      throw std::runtime_error("Invalid transform value at position " +
                              std::to_string(idx) + ": " + value);
    }
  }

  if (idx != 16) {
    throw std::runtime_error("Transform matrix must have exactly 16 values, got " +
                            std::to_string(idx));
  }

  return transform;
}

// Register this stage with the factory
REGISTER_STAGE(TransformStage)

} // namespace height_mapping::core