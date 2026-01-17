/*
 * voxel_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_STAGES_VOXEL_FILTER_H
#define HEIGHT_MAPPING_LAB_STAGES_VOXEL_FILTER_H

#include <spdlog/spdlog.h>

#include <memory>
#include <nanopcl/filters/downsample.hpp>

#include "height_mapping/lab/frame.h"

namespace height_mapping::lab::stages {

/**
 * @brief Pipeline stage adapter for voxel filter (header-only)
 */
class VoxelFilter : public ::ppl::Stage<MappingFrame> {
  using VoxelMethod = npcl::filters::VoxelMethod;

 public:
  VoxelFilter() = default;

  void configure(const YAML::Node& config) override {
    if (config["voxel_size"]) {
      voxel_size_ = config["voxel_size"].as<float>();
    }

    if (config["method"]) {
      std::string method = config["method"].as<std::string>();
      if (method == "centroid") {
        method_ = VoxelMethod::CENTROID;
      } else if (method == "arbitrary") {
        method_ = VoxelMethod::ARBITRARY;
      } else if (method == "first") {
        method_ = VoxelMethod::FIRST;
      } else if (method == "closest") {
        method_ = VoxelMethod::CLOSEST;
      } else if (method == "center" || method == "voxel_center") {
        method_ = VoxelMethod::VOXEL_CENTER;
      } else {
        spdlog::error("[VoxelFilter] Unknown reduction method: {}", method);
      }
    }

    spdlog::debug("[VoxelFilter] voxel_size={}m", voxel_size_);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;

    if (cloud.empty()) {
      return true;
    }

    size_t before = cloud.size();
    cloud = npcl::filters::voxelGrid(std::move(cloud), voxel_size_, method_);

    spdlog::debug("[VoxelFilter] {} -> {} points", before, cloud.size());
    return true;
  }

 private:
  float voxel_size_ = 0.1f;
  VoxelMethod method_ = VoxelMethod::CENTROID;
};

}  // namespace height_mapping::lab::stages
#endif  // HEIGHT_MAPPING_LAB_STAGES_VOXEL_FILTER_H
