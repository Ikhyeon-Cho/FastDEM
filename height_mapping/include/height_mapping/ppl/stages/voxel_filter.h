/*
 * voxel_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_STAGES_VOXEL_FILTER_H
#define HEIGHT_MAPPING_PPL_STAGES_VOXEL_FILTER_H

#include <spdlog/spdlog.h>

#include <memory>

#include <nanopcl/filters/voxel_grid.hpp>
#include "height_mapping/ppl/types.h"

namespace height_mapping::ppl::stages {

/**
 * @brief Pipeline stage adapter for voxel filter (header-only)
 */
class VoxelFilter : public ::ppl::Stage<MappingFrame> {
 public:
  VoxelFilter() : filter_(std::make_unique<nanopcl::filters::VoxelGrid>()) {}

  void configure(const YAML::Node& config) override {
    nanopcl::filters::VoxelGrid::Config cfg;

    // Parse voxel size
    if (config["voxel_size"]) {
      cfg.voxel_size = config["voxel_size"].as<float>();
    }

    // Parse reduction method
    if (config["reduction_method"]) {
      std::string method = config["reduction_method"].as<std::string>();
      if (method == "centroid") {
        cfg.method = nanopcl::filters::VoxelGrid::Method::CENTROID;
      } else if (method == "random") {
        cfg.method = nanopcl::filters::VoxelGrid::Method::RANDOM;
      } else if (method == "first") {
        cfg.method = nanopcl::filters::VoxelGrid::Method::FIRST;
      } else if (method == "closest") {
        cfg.method = nanopcl::filters::VoxelGrid::Method::CLOSEST;
      } else if (method == "center" || method == "voxel_center") {
        cfg.method = nanopcl::filters::VoxelGrid::Method::VOXEL_CENTER;
      } else {
        spdlog::error("[VoxelFilter] Unknown reduction method: {}", method);
      }
    }

    filter_->setConfig(cfg);
    spdlog::debug("[VoxelFilter] voxel_size={}m", cfg.voxel_size);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;

    if (cloud.empty()) {
      return true;
    }

    // Run the core algorithm in-place
    auto stats = filter_->filterInPlace(cloud);

    spdlog::debug("[VoxelFilter] Downsampled from {} to {} points ({} voxels)",
                  stats.input_size, stats.output_size, stats.voxel_count);
    return true;
  }

 private:
  std::unique_ptr<nanopcl::filters::VoxelGrid> filter_;
};

}  // namespace height_mapping::ppl::stages
#endif  // HEIGHT_MAPPING_PPL_STAGES_VOXEL_FILTER_H
