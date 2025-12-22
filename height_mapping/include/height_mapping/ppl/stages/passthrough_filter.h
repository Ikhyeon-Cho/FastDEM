/*
 * passthrough_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_STAGES_PASSTHROUGH_FILTER_H
#define HEIGHT_MAPPING_PPL_STAGES_PASSTHROUGH_FILTER_H

#include <spdlog/spdlog.h>

#include <limits>
#include <memory>

#include <nanopcl/filters/passthrough.hpp>
#include <nanopcl/transform/operations.hpp>
#include "height_mapping/ppl/types.h"

namespace height_mapping::ppl::stages {

/**
 * @brief Pipeline stage adapter for passthrough filter (header-only)
 */
class PassthroughFilter : public ::ppl::Stage<MappingFrame> {
 public:
  PassthroughFilter()
      : filter_(std::make_unique<nanopcl::filters::Passthrough>()) {}

  void configure(const YAML::Node& config) override {
    nanopcl::filters::Passthrough::Config cfg;

    // Parse axis-aligned bounds
    if (config["x_min"]) cfg.x_min = config["x_min"].as<float>();
    if (config["x_max"]) cfg.x_max = config["x_max"].as<float>();
    if (config["y_min"]) cfg.y_min = config["y_min"].as<float>();
    if (config["y_max"]) cfg.y_max = config["y_max"].as<float>();
    if (config["z_min"]) cfg.z_min = config["z_min"].as<float>();
    if (config["z_max"]) cfg.z_max = config["z_max"].as<float>();

    // Parse distance bounds
    if (config["distance_min"]) {
      cfg.distance_min = config["distance_min"].as<float>();
      cfg.use_distance_filter = true;
    }
    if (config["distance_max"]) {
      cfg.distance_max = config["distance_max"].as<float>();
      cfg.use_distance_filter = true;
    }

    filter_->setConfig(cfg);

    // Format bounds for logging (show "inf" for limit values)
    auto format_bound = [](float value) -> std::string {
      if (value <= std::numeric_limits<float>::lowest()) return "-inf";
      if (value >= std::numeric_limits<float>::max()) return "+inf";
      return std::to_string(static_cast<int>(value));
    };

    spdlog::debug("[PassthroughFilter] bounds [{},{}] x [{},{}] x [{},{}]",
                  format_bound(cfg.x_min), format_bound(cfg.x_max),
                  format_bound(cfg.y_min), format_bound(cfg.y_max),
                  format_bound(cfg.z_min), format_bound(cfg.z_max));
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;

    if (cloud.empty()) {
      return true;
    }

    // Transform to base frame if needed (passthrough operates in base frame)
    const auto& T_base_sensor = frame->extrinsic;
    nanopcl::transformInPlace(cloud, T_base_sensor);

    // Run the core algorithm in-place
    auto stats = filter_->filterInPlace(cloud);

    spdlog::debug("[PassthroughFilter] Removed {} points ({} -> {})",
                  stats.removed_count, stats.input_size, stats.output_size);
    return true;
  }

 private:
  std::unique_ptr<nanopcl::filters::Passthrough> filter_;
};

}  // namespace height_mapping::ppl::stages
#endif  // HEIGHT_MAPPING_PPL_STAGES_PASSTHROUGH_FILTER_H
