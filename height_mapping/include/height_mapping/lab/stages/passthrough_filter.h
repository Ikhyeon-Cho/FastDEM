/*
 * passthrough_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_STAGES_PASSTHROUGH_FILTER_H
#define HEIGHT_MAPPING_LAB_STAGES_PASSTHROUGH_FILTER_H

#include <spdlog/spdlog.h>

#include <limits>
#include <memory>
#include <nanopcl/filters/crop.hpp>
#include <nanopcl/transform/transform_ops.hpp>

#include "height_mapping/lab/frame.h"

namespace height_mapping::lab::stages {

/**
 * @brief Pipeline stage for spatial filtering (z-axis + range)
 *
 * Uses cropZaxis and cropRange from nanoPCL filter API.
 */
class PassthroughFilter : public MappingStage {
 public:
  PassthroughFilter() = default;

  void configure(const YAML::Node& config) override {
    if (config["z_min"]) z_min_ = config["z_min"].as<float>();
    if (config["z_max"]) z_max_ = config["z_max"].as<float>();
    if (config["range_min"]) range_min_ = config["range_min"].as<float>();
    if (config["range_max"]) range_max_ = config["range_max"].as<float>();

    spdlog::debug("[PassthroughFilter] z=[{},{}], range=[{},{}]", z_min_,
                  z_max_, range_min_, range_max_);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;
    const auto& T_base_sensor = frame->extrinsic;

    if (cloud.empty()) {
      return true;
    }

    // Filter operates in base frame
    if (T_base_sensor.isValid() &&
        cloud.frameId() != T_base_sensor.parentFrame()) {
      cloud = npcl::transformCloud(std::move(cloud), T_base_sensor);
    }

    size_t before = cloud.size();

    // Apply z-axis filter
    cloud = npcl::filters::cropZaxis(std::move(cloud), z_min_, z_max_);

    // Apply range filter
    cloud = npcl::filters::cropRange(std::move(cloud), range_min_, range_max_);

    spdlog::debug("[PassthroughFilter] {} -> {} points", before, cloud.size());
    return true;
  }

 private:
  float z_min_ = -std::numeric_limits<float>::max();
  float z_max_ = std::numeric_limits<float>::max();
  float range_min_ = 0.0f;
  float range_max_ = std::numeric_limits<float>::max();
};

}  // namespace height_mapping::lab::stages
#endif  // HEIGHT_MAPPING_LAB_STAGES_PASSTHROUGH_FILTER_H
