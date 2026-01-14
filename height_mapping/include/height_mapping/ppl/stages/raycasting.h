/*
 * raycasting.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_STAGES_RAYCASTING_H
#define HEIGHT_MAPPING_PPL_STAGES_RAYCASTING_H

#include <spdlog/spdlog.h>

#include <memory>
#include <nanopcl/transform/transform_ops.hpp>

#include "height_mapping/algorithms/algorithms.h"
#include "height_mapping/ppl/types.h"

namespace height_mapping::ppl::stages {

/**
 * @brief Pipeline stage adapter for raycasting correction (header-only)
 *
 * This stage optionally uses IRobotPoseProvider service to get sensor origin.
 * Note: Current implementation uses robot pose as approximate sensor origin.
 * TODO: Fix this to properly compute sensor origin using extrinsics.
 */
class Raycasting : public ::ppl::Stage<MappingFrame> {
 public:
  Raycasting() = default;

  void configure(const YAML::Node& config) override {
    if (config["correction_threshold"])
      config_.correction_threshold = config["correction_threshold"].as<float>();
    if (config["min_ray_distance"])
      config_.min_ray_distance = config["min_ray_distance"].as<float>();
    if (config["enable_correction"])
      config_.enable_correction = config["enable_correction"].as<bool>();

    // Persistence parameters
    if (config["persistence_max_count"])
      config_.persistence.max_count =
          config["persistence_max_count"].as<float>();
    if (config["persistence_step_add"])
      config_.persistence.step_add = config["persistence_step_add"].as<float>();
    if (config["persistence_step_sub"])
      config_.persistence.step_sub = config["persistence_step_sub"].as<float>();

    spdlog::debug("[Raycasting] threshold={}m, persistence=[max:{}, +{}, -{}]",
                  config_.correction_threshold, config_.persistence.max_count,
                  config_.persistence.step_add, config_.persistence.step_sub);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;
    auto& map = frame->height_map;

    if (!map || cloud.empty() || !config_.enable_correction) {
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
    algorithms::applyRaycasting(*map, cloud, sensor_origin, config_);

    return true;
  }

 private:
  algorithms::RaycastingConfig config_;
};

}  // namespace height_mapping::ppl::stages
#endif  // HEIGHT_MAPPING_PPL_STAGES_RAYCASTING_H
