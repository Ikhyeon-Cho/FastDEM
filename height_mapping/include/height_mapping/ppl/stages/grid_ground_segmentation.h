/*
 * grid_ground_segmentation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_STAGES_GRID_GROUND_SEGMENTATION_H
#define HEIGHT_MAPPING_PPL_STAGES_GRID_GROUND_SEGMENTATION_H

#include <spdlog/spdlog.h>

#include <memory>

#include "height_mapping/algorithms/algorithms.h"
#include "height_mapping/ppl/types.h"

namespace height_mapping::ppl::stages {

/**
 * @brief Pipeline stage adapter for grid-based ground segmentation
 * (header-only)
 *
 * This stage wraps the core GridGroundSegmentation algorithm for use in the
 * pipeline. It handles configuration parsing from YAML and context management.
 */
class GridGroundSegmentation : public ::ppl::Stage<MappingFrame> {
 public:
  GridGroundSegmentation() = default;

  void configure(const YAML::Node& config) override {
    // Parse parameters - only update if present in config
    if (config["grid_resolution"])
      config_.grid_resolution = config["grid_resolution"].as<float>();
    if (config["cell_percentile"])
      config_.cell_percentile = config["cell_percentile"].as<float>();
    if (config["ground_thickness"])
      config_.ground_thickness = config["ground_thickness"].as<float>();
    if (config["max_ground_height"])
      config_.max_ground_height = config["max_ground_height"].as<float>();
    if (config["min_points_per_cell"])
      config_.min_points_per_cell = config["min_points_per_cell"].as<int>();
    if (config["keep_only_ground"])
      config_.keep_only_ground = config["keep_only_ground"].as<bool>();

    spdlog::debug(
        "[GridGroundSegmentation] grid_resolution={}m, "
        "cell_percentile={}, ground_thickness={}m",
        config_.grid_resolution, config_.cell_percentile, config_.ground_thickness);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;

    if (cloud.empty()) {
      spdlog::warn("[GridGroundSegmentation] Input cloud is empty, skipping");
      return true;
    }

    // Run the core algorithm in-place
    algorithms::segmentGroundGrid(cloud, config_);

    return true;
  }

 private:
  algorithms::GridGroundSegmentationConfig config_;
};

}  // namespace height_mapping::ppl::stages
#endif  // HEIGHT_MAPPING_PPL_STAGES_GRID_GROUND_SEGMENTATION_H
