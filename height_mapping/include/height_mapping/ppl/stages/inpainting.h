/*
 * inpainting.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_STAGES_INPAINTING_H
#define HEIGHT_MAPPING_PPL_STAGES_INPAINTING_H

#include <spdlog/spdlog.h>

#include "height_mapping/algorithms/algorithms.h"
#include "height_mapping/ppl/types.h"

namespace height_mapping::ppl::stages {

/**
 * @brief Pipeline stage for inpainting NaN holes in height map
 */
class Inpainting : public ::ppl::Stage<MappingFrame> {
 public:
  Inpainting() = default;

  void configure(const YAML::Node& config) override {
    if (config["enabled"]) {
      config_.enabled = config["enabled"].as<bool>();
    }
    if (config["max_iterations"]) {
      config_.max_iterations = config["max_iterations"].as<int>();
    }
    if (config["min_valid_neighbors"]) {
      config_.min_valid_neighbors = config["min_valid_neighbors"].as<int>();
    }

    spdlog::debug("[Inpainting] enabled={}, max_iter={}, min_neighbors={}",
                  config_.enabled, config_.max_iterations,
                  config_.min_valid_neighbors);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    if (!config_.enabled) {
      return true;
    }

    auto& map = frame->height_map;
    if (!map) {
      throw std::runtime_error("[Inpainting] HeightMap not available");
    }

    algorithms::applyInpainting(*map, config_);

    return true;
  }

 private:
  algorithms::InpaintingConfig config_;
};

}  // namespace height_mapping::ppl::stages

#endif  // HEIGHT_MAPPING_PPL_STAGES_INPAINTING_H
