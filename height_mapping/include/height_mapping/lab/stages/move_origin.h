/*
 * move_origin.h
 *
 * Pipeline stage for robot-centric mapping.
 * Include this stage to make the map follow the robot.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_STAGES_MOVE_ORIGIN_H
#define HEIGHT_MAPPING_LAB_STAGES_MOVE_ORIGIN_H

#include <spdlog/spdlog.h>

#include "height_mapping/lab/frame.h"

namespace height_mapping::lab::stages {

/**
 * @brief Pipeline stage that moves map origin to robot position
 *
 * Include this stage in the pipeline for robot-centric (local) mapping.
 * Exclude it for map-centric (global) mapping.
 */
class MoveOrigin : public ::ppl::Stage<MappingFrame> {
 public:
  MoveOrigin() = default;

  void configure(const YAML::Node& /*config*/) override {
    spdlog::debug("[MoveOrigin] Stage configured (robot-centric mode)");
  }

  bool process(const std::shared_ptr<MappingFrame>& dataframe) override {
    auto& map = dataframe->height_map;
    if (!map) return true;

    // Move map origin to robot position
    grid_map::Position robot_pos(dataframe->robot_pose.translation().x(),
                                 dataframe->robot_pose.translation().y());
    map->move(robot_pos);

    spdlog::debug("[MoveOrigin] Map origin moved to ({:.2f}, {:.2f})",
                  robot_pos.x(), robot_pos.y());
    return true;
  }
};

}  // namespace height_mapping::lab::stages
#endif  // HEIGHT_MAPPING_LAB_STAGES_MOVE_ORIGIN_H
