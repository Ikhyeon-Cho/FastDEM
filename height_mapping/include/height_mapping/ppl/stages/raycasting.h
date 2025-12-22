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

#include <cmath>
#include <memory>

#include "height_mapping/interfaces/robot_pose_provider.h"
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
    // Parse max ground angle (convert from degrees to radians)
    if (config["max_ground_angle"]) {
      float angle_degrees = config["max_ground_angle"].as<float>();
      config_.max_ground_angle = angle_degrees * M_PI / 180.0f;
    }

    // Parse other parameters
    if (config["correction_threshold"])
      config_.correction_threshold = config["correction_threshold"].as<float>();
    if (config["enable_correction"])
      config_.enable_correction = config["enable_correction"].as<bool>();
    if (config["ray_step_size"])
      config_.ray_step_size = config["ray_step_size"].as<float>();

    spdlog::debug("[Raycasting] max_ground_angle={} degrees, threshold={}m",
                  config_.max_ground_angle * 180.0f / M_PI,
                  config_.correction_threshold);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;
    auto& map = frame->map;

    if (!map || cloud.empty() || !config_.enable_correction) {
      return true;
    }

    // Compute sensor origin in map frame
    const auto& T_map_base = frame->pose;
    const auto& T_base_sensor = frame->extrinsic;
    Eigen::Vector3f sensor_origin = (T_map_base * T_base_sensor).translation();

    // Run the core algorithm
    algorithms::applyRaycasting(*map, cloud, sensor_origin, config_);

    return true;
  }

 private:
  algorithms::RaycastingConfig config_;
};

}  // namespace height_mapping::ppl::stages
#endif  // HEIGHT_MAPPING_PPL_STAGES_RAYCASTING_H
