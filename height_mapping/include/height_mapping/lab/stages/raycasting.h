/*
 * raycasting.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_STAGES_RAYCASTING_H
#define HEIGHT_MAPPING_LAB_STAGES_RAYCASTING_H

#include <spdlog/spdlog.h>

#include <memory>
#include <nanopcl/transform/transform_ops.hpp>

#include "height_mapping/config/raycasting.h"
#include "height_mapping/lab/frame.h"
#include "height_mapping/ops/raycasting.h"

namespace height_mapping::lab::stages {

/**
 * @brief Pipeline stage adapter for raycasting postprocess (header-only)
 *
 * This stage optionally uses IRobotPoseProvider service to get sensor origin.
 * Note: Current implementation uses robot pose as approximate sensor origin.
 * TODO: Fix this to properly compute sensor origin using extrinsics.
 */
class Raycasting : public ::ppl::Stage<MappingFrame> {
 public:
  Raycasting() = default;

  void configure(const YAML::Node& config) override {
    if (config["enabled"]) config_.enabled = config["enabled"].as<bool>();
    if (config["endpoint_margin"])
      config_.endpoint_margin = config["endpoint_margin"].as<int>();
    if (config["height_threshold"])
      config_.height_threshold = config["height_threshold"].as<float>();
    if (config["vote_threshold"])
      config_.vote_threshold = config["vote_threshold"].as<int>();

    spdlog::debug(
        "[Raycasting] height_threshold={}m, vote_threshold={}, endpoint_margin={}",
        config_.height_threshold, config_.vote_threshold, config_.endpoint_margin);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;
    auto& map = frame->height_map;

    if (!map || cloud.empty() || !config_.enabled) {
      return true;
    }

    // Transform to map frame if needed
    if (frame->robot_pose.isValid() &&
        cloud.frameId() != frame->robot_pose.parentFrame()) {
      cloud = npcl::transformCloud(std::move(cloud), frame->robot_pose);
    }

    // Compute sensor origin in map frame
    Eigen::Vector3f sensor_origin =
        (frame->robot_pose * frame->extrinsic).translation();

    // Run the core algorithm
    ops::applyRaycasting(*map, cloud, sensor_origin, config_);

    return true;
  }

 private:
  config::Raycasting config_;
};

}  // namespace height_mapping::lab::stages
#endif  // HEIGHT_MAPPING_LAB_STAGES_RAYCASTING_H
