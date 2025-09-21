/*
 * map_origin_update.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/map_origin_update.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <iostream>
#include <cmath>

namespace height_mapping::core {

MapOriginUpdateStage::MapOriginUpdateStage()
    : Stage("MapOriginUpdate", "Map Origin Controller") {
}

void MapOriginUpdateStage::configure(const std::map<std::string, std::string>& params) {
  auto it = params.find("update_mode");
  if (it != params.end()) {
    const std::string& mode = it->second;
    if (mode == "continuous") {
      update_mode_ = UpdateMode::CONTINUOUS;
    } else if (mode == "threshold") {
      update_mode_ = UpdateMode::THRESHOLD;
    } else if (mode == "centered") {
      update_mode_ = UpdateMode::CENTERED;
    } else if (mode == "fixed") {
      update_mode_ = UpdateMode::FIXED;
    } else {
      std::cerr << "[MapOriginUpdate] Unknown update mode: " << mode
                << ", using CONTINUOUS" << std::endl;
    }
  }

  it = params.find("update_threshold");
  if (it != params.end()) {
    update_threshold_ = std::stof(it->second);
  }

  it = params.find("center_region_size");
  if (it != params.end()) {
    center_region_size_ = std::stof(it->second);
  }

  it = params.find("robot_frame");
  if (it != params.end()) {
    robot_frame_ = it->second;
  }

  it = params.find("map_frame");
  if (it != params.end()) {
    map_frame_ = it->second;
  }
}

void MapOriginUpdateStage::processImpl(pipeline::Context& ctx) {
  auto& mapping_ctx = static_cast<MappingContext&>(ctx);

  auto& map = mapping_ctx.map();
  if (map.empty()) {
    std::cerr << "[MapOriginUpdate] No height map in context" << std::endl;
    return;
  }

  // Get robot position (simplified - would come from transform in real implementation)
  Eigen::Vector3f robot_position = Eigen::Vector3f::Zero();

  // In a real implementation, this would come from TF or another source
  // For now, use a default position

  bool should_update = false;
  std::string update_reason = "none";

  // Determine if we should update the map origin
  switch (update_mode_) {
    case UpdateMode::CONTINUOUS:
      should_update = true;
      update_reason = "continuous";
      break;

    case UpdateMode::THRESHOLD:
      if (first_update_) {
        should_update = true;
        update_reason = "initial";
      } else {
        float distance = (robot_position - last_update_position_).norm();
        if (distance > update_threshold_) {
          should_update = true;
          update_reason = "threshold_exceeded";
          total_distance_moved_ += distance;
        }
      }
      break;

    case UpdateMode::CENTERED:
      {
        // Check if robot is outside center region
        float dist_from_center = robot_position.head<2>().norm();
        if (dist_from_center > center_region_size_ / 2.0f) {
          should_update = true;
          update_reason = "outside_center_region";
        }
      }
      break;

    case UpdateMode::FIXED:
      // Never update
      break;
  }

  if (should_update) {
    // Here we would actually update the map origin
    // This is a simplified placeholder

    // Track the update
    last_update_position_ = robot_position;
    first_update_ = false;
    total_updates_++;

    // Store in context
  } else {
  }

  // Always store statistics
}

// Register this stage with the factory
REGISTER_STAGE(MapOriginUpdateStage)

} // namespace height_mapping::core