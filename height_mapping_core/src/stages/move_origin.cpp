/*
 * move_origin.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/move_origin.h"
#include "height_mapping_core/interfaces/transform_provider.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registration.h"

#include <cmath>
#include <logger/logger.h>

namespace height_mapping::core::stages {

MoveOrigin::MoveOrigin() : Stage("MoveOrigin") {}

void MoveOrigin::configure(const std::map<std::string, std::string> &params) {
  auto it = params.find("update_mode");
  if (it != params.end()) {
    const std::string &mode = it->second;
    if (mode == "continuous") {
      update_mode_ = UpdateMode::CONTINUOUS;
    } else if (mode == "threshold") {
      update_mode_ = UpdateMode::THRESHOLD;
    } else if (mode == "centered") {
      update_mode_ = UpdateMode::CENTERED;
    } else if (mode == "fixed") {
      update_mode_ = UpdateMode::FIXED;
    } else {
      LOG_ERROR("MoveOrigin", "Unknown update mode: ", mode,
                ", using CONTINUOUS");
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

void MoveOrigin::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  auto &map = mapping_ctx.map();
  auto &cloud = mapping_ctx.cloud();

  // Get transform provider to lookup robot position
  auto transform_provider = ctx.getService<ITransformProvider>();
  if (!transform_provider) {
    // If no transform provider, can't update map origin
    return;
  }

  // Get robot position in map frame using cloud timestamp
  // This ensures temporal consistency with the sensor data
  auto transform_opt = transform_provider->lookupTransform(
      map_frame_, robot_frame_, cloud.timestamp());

  if (!transform_opt.has_value()) {
    // Can't get robot position, skip update
    return;
  }

  // Extract robot position from transform
  const auto &T = transform_opt.value().matrix();
  Eigen::Vector3f robot_position(T(0, 3), T(1, 3), 0.0f); // Only care about x,y

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

  case UpdateMode::CENTERED: {
    // Check if robot is outside center region
    float dist_from_center = robot_position.head<2>().norm();
    if (dist_from_center > center_region_size_ / 2.0f) {
      should_update = true;
      update_reason = "outside_center_region";
    }
  } break;

  case UpdateMode::FIXED:
    // Never update
    break;
  }

  if (should_update) {
    // Move the map to be centered on the robot
    grid_map::Position new_position(robot_position.x(), robot_position.y());

    // Move the map - this shifts the grid to the new position
    // GridMap::move() keeps the data and shifts the coordinate frame
    map.move(new_position);

    // Track the update
    last_update_position_ = robot_position;
    first_update_ = false;
    total_updates_++;
  }
}

// Register this stage with the factory
REGISTER_STAGE(MoveOrigin)

} // namespace height_mapping::core::stages