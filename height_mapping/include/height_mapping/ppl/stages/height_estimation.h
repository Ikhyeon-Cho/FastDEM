/*
 * height_estimation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_STAGES_HEIGHT_ESTIMATION_H
#define HEIGHT_MAPPING_PPL_STAGES_HEIGHT_ESTIMATION_H

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

#include <nanopcl/transform/transform_ops.hpp>

#include "height_mapping/algorithms/algorithms.h"
#include "height_mapping/estimators/height_estimator_base.h"
#include "height_mapping/estimators/incremental_mean.h"
#include "height_mapping/estimators/kalman_filter.h"
#include "height_mapping/estimators/moving_average.h"
#include "height_mapping/ppl/types.h"

namespace height_mapping::ppl::stages {

/**
 * @brief Pipeline stage adapter for height estimation (header-only)
 */
class HeightEstimation : public ::ppl::Stage<MappingFrame> {
 public:
  HeightEstimation() = default;

  void configure(const YAML::Node& config) override {
    std::string type = "incremental_mean";
    if (config["estimator_type"]) {
      type = config["estimator_type"].as<std::string>();
    }

    if (type == "kalman_filter") {
      estimators::KalmanFilter::Parameters params;
      if (config["kalman_process_noise"])
        params.process_noise = config["kalman_process_noise"].as<float>();
      if (config["kalman_measurement_noise"])
        params.measurement_noise = config["kalman_measurement_noise"].as<float>();
      if (config["kalman_initial_variance"])
        params.initial_variance = config["kalman_initial_variance"].as<float>();

      estimator_ = std::make_unique<estimators::KalmanFilter>(params);

    } else if (type == "moving_average") {
      estimators::MovingAverage::Parameters params;
      if (config["moving_average_alpha"])
        params.alpha = config["moving_average_alpha"].as<float>();

      estimator_ = std::make_unique<estimators::MovingAverage>(params);

    } else {
      if (type != "incremental_mean") {
        spdlog::error(
            "[HeightEstimation] Unknown estimator type: {}, using incremental_mean",
            type);
      }
      estimator_ = std::make_unique<estimators::IncrementalMean>();
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
      estimator_ = std::make_unique<estimators::IncrementalMean>();
    }

    // Run the core algorithm
    algorithms::updateHeightMap(*map, cloud, *estimator_);

    return true;
  }

 private:
  std::unique_ptr<estimators::HeightEstimatorBase> estimator_;
};

}  // namespace height_mapping::ppl::stages
#endif  // HEIGHT_MAPPING_PPL_STAGES_HEIGHT_ESTIMATION_H
