/*
 * statistical_ground_segmentation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_STAGES_STATISTICAL_GROUND_SEGMENTATION_H
#define HEIGHT_MAPPING_PPL_STAGES_STATISTICAL_GROUND_SEGMENTATION_H

#include <spdlog/spdlog.h>

#include "height_mapping/algorithms/algorithms.h"
#include "height_mapping/ppl/types.h"

namespace height_mapping::ppl::stages {

/**
 * @brief Pipeline stage adapter for statistical ground segmentation
 */
class StatisticalGroundSegmentation : public ::ppl::Stage<MappingFrame> {
 public:
  void configure(const YAML::Node& config) override {
    if (config["ground_percentile"])
      config_.ground_percentile = config["ground_percentile"].as<float>();
    if (config["ground_thickness"])
      config_.ground_thickness = config["ground_thickness"].as<float>();
    if (config["noise_threshold"])
      config_.noise_threshold = config["noise_threshold"].as<float>();
    if (config["keep_only_ground"])
      config_.keep_only_ground = config["keep_only_ground"].as<bool>();

    spdlog::debug(
        "[StatisticalGroundSegmentation] percentile={}, thickness={}m",
        config_.ground_percentile, config_.ground_thickness);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;

    if (cloud.empty()) {
      spdlog::warn(
          "[StatisticalGroundSegmentation] Input cloud is empty, skipping");
      return true;
    }

    auto stats = algorithms::segmentGroundStatistical(cloud, config_);

    spdlog::debug(
        "[StatisticalGroundSegmentation] {} ground, {} obstacle, {} noise "
        "points ({}th percentile z={}m)",
        stats.ground_count, stats.obstacle_count, stats.noise_count,
        static_cast<int>(config_.ground_percentile * 100), stats.robust_min_z);
    return true;
  }

 private:
  algorithms::StatisticalGroundSegmentationConfig config_;
};

}  // namespace height_mapping::ppl::stages
#endif  // HEIGHT_MAPPING_PPL_STAGES_STATISTICAL_GROUND_SEGMENTATION_H
