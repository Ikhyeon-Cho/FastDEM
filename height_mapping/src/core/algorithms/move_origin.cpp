/*
 * move_origin.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/core/algorithms/move_origin.h"

#include <cmath>

namespace height_mapping::algorithms {

MoveOrigin::Stats MoveOrigin::updateOrigin(
    height_mapping::HeightMap &map, const Eigen::Vector3f &robot_position) {
  Stats stats;
  stats.old_position = last_update_position_;
  stats.total_updates = cumulative_stats_.total_updates;
  stats.total_distance = cumulative_stats_.total_distance;

  // Determine if we should update
  if (!shouldUpdate(robot_position, stats.update_reason)) {
    stats.updated = false;
    return stats;
  }

  // Move the map to be centered on the robot
  grid_map::Position new_position(robot_position.x(), robot_position.y());

  // Move the map - this shifts the grid to the new position
  // GridMap::move() keeps the data and shifts the coordinate frame
  map.move(new_position);

  // Calculate distance moved
  if (!first_update_) {
    stats.distance_moved = (robot_position - last_update_position_).norm();
    cumulative_stats_.total_distance += stats.distance_moved;
  }

  // Update tracking state
  stats.updated = true;
  stats.new_position = robot_position;
  last_update_position_ = robot_position;
  first_update_ = false;
  cumulative_stats_.total_updates++;

  // Update cumulative stats
  stats.total_updates = cumulative_stats_.total_updates;
  stats.total_distance = cumulative_stats_.total_distance;

  return stats;
}

bool MoveOrigin::shouldUpdate(const Eigen::Vector3f &robot_position,
                              std::string &reason) {
  switch (config_.update_mode) {
    case UpdateMode::CONTINUOUS:
      reason = "continuous";
      return true;

    case UpdateMode::THRESHOLD:
      if (first_update_) {
        reason = "initial";
        return true;
      } else {
        float distance = (robot_position - last_update_position_).norm();
        if (distance > config_.update_threshold) {
          reason = "threshold_exceeded";
          return true;
        }
      }
      break;

    case UpdateMode::CENTERED: {
      // Check if robot is outside center region
      float dist_from_center = robot_position.head<2>().norm();
      if (dist_from_center > config_.center_region_size / 2.0f) {
        reason = "outside_center_region";
        return true;
      }
    } break;

    case UpdateMode::FIXED:
      reason = "fixed_mode";
      return false;
  }

  reason = "no_update_needed";
  return false;
}

}  // namespace height_mapping::algorithms