/*
 * voxel_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_VOXEL_FILTER_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_VOXEL_FILTER_H

#include "height_mapping_core/algorithms/voxel_filter.h"
#include "height_mapping_pipeline/mapping_context.h"
#include "flowpipe/stage.h"
#include "flowpipe/stage_registration.h"
#include <logger/logger.h>
#include <memory>

namespace height_mapping::mapping::stages {

/**
 * @brief Pipeline stage adapter for voxel filter (header-only)
 */
class VoxelFilter : public flowpipe::Stage {
public:
  VoxelFilter()
      : Stage("VoxelFilter"),
        filter_(std::make_unique<algorithms::VoxelFilter>()) {}

  void configure(const std::map<std::string, std::string> &params) override {
    algorithms::VoxelFilter::Config config;

    // Parse voxel size
    loadParam(params, "voxel_size", config.voxel_size);

    // Parse reduction method
    std::string method;
    if (loadParam(params, "reduction_method", method)) {
      if (method == "centroid") {
        config.reduction_method =
            algorithms::VoxelFilter::ReductionMethod::CENTROID;
      } else if (method == "random") {
        config.reduction_method =
            algorithms::VoxelFilter::ReductionMethod::RANDOM;
      } else if (method == "first") {
        config.reduction_method =
            algorithms::VoxelFilter::ReductionMethod::FIRST;
      } else if (method == "closest") {
        config.reduction_method =
            algorithms::VoxelFilter::ReductionMethod::CLOSEST;
      } else if (method == "center" || method == "voxel_center") {
        config.reduction_method =
            algorithms::VoxelFilter::ReductionMethod::VOXEL_CENTER;
      } else {
        LOG_ERROR(getName(), "Unknown reduction method: ", method);
      }
    }

    // Update or create filter with new config
    if (filter_) {
      filter_->setConfig(config);
    } else {
      filter_ = std::make_unique<algorithms::VoxelFilter>(config);
    }

    LOG_DEBUG(getName(), "Configured with voxel_size=", config.voxel_size, "m");
  }

protected:
  void processImpl(flowpipe::Context &ctx) override {
    auto &mapping_ctx = static_cast<MappingContext &>(ctx);
    auto &cloud = mapping_ctx.cloud();

    if (cloud.empty()) {
      return;
    }

    // Run the core algorithm in-place
    auto stats = filter_->filterInPlace(cloud);

    LOG_DEBUG_THROTTLE(1.0, getName(), "VoxelFilter downsampled from ",
                       stats.original_size, " to ", stats.filtered_size,
                       " points (", stats.voxel_count, " voxels)");
  }

private:
  std::unique_ptr<algorithms::VoxelFilter> filter_;
};

REGISTER_STAGE(VoxelFilter)

} // namespace height_mapping::mapping::stages
#endif // HEIGHT_MAPPING_PIPELINE_STAGES_VOXEL_FILTER_H