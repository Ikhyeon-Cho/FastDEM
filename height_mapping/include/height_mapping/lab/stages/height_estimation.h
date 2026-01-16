/*
 * height_estimation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_STAGES_HEIGHT_ESTIMATION_H
#define HEIGHT_MAPPING_LAB_STAGES_HEIGHT_ESTIMATION_H

#include <spdlog/spdlog.h>

#include <memory>
#include <nanopcl/transform/transform_ops.hpp>
#include <string>

#include "height_mapping/core.h"
#include "height_mapping/lab/frame.h"

namespace height_mapping::lab::stages {

/**
 * @brief Pipeline stage adapter for height estimation (header-only)
 */
class HeightEstimation : public ::ppl::Stage<MappingFrame> {
 public:
  HeightEstimation() = default;

  void configure(const YAML::Node& config) override {
    std::string type = "incremental_mean";
    if (config["type"]) {
      type = config["type"].as<std::string>();
    }

    if (type == "kalman_filter") {
      float process_noise = 0.01f;
      float measurement_noise = 0.1f;
      float initial_variance = 1.0f;
      if (auto k = config["kalman"]) {
        if (k["process_noise"]) process_noise = k["process_noise"].as<float>();
        if (k["measurement_noise"])
          measurement_noise = k["measurement_noise"].as<float>();
        if (k["initial_variance"])
          initial_variance = k["initial_variance"].as<float>();
      }

      estimator_ = std::make_unique<estimator::KalmanFilter>(
          process_noise, measurement_noise, initial_variance);

    } else if (type == "moving_average") {
      float alpha = 0.3f;
      if (auto m = config["moving_average"]) {
        if (m["alpha"]) alpha = m["alpha"].as<float>();
      }

      estimator_ = std::make_unique<estimator::MovingAverage>(alpha);

    } else {
      if (type != "incremental_mean") {
        spdlog::error(
            "[HeightEstimation] Unknown estimator type: {}, using "
            "incremental_mean",
            type);
      }
      estimator_ = std::make_unique<estimator::IncrementalMean>();
    }

    spdlog::debug("[HeightEstimation] estimator={}", type);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;
    auto& map = frame->height_map;

    if (!map) {
      throw std::runtime_error("[HeightEstimation] HeightMap not available");
    }

    if (cloud.empty()) {
      return true;
    }

    // Transform to map frame if needed
    if (frame->robot_pose.isValid() &&
        cloud.frameId() != frame->robot_pose.parentFrame()) {
      cloud = npcl::transformCloud(std::move(cloud), frame->robot_pose);
    }

    if (!estimator_) {
      estimator_ = std::make_unique<estimator::IncrementalMean>();
    }

    // Run the core algorithm
    updater::Elevation elevation(*estimator_);
    height_mapping::update(*map, cloud, elevation);

    return true;
  }

 private:
  std::unique_ptr<estimator::Base> estimator_;
};

}  // namespace height_mapping::lab::stages
#endif  // HEIGHT_MAPPING_LAB_STAGES_HEIGHT_ESTIMATION_H
