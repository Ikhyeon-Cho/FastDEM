/*
 * passthrough_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_PASSTHROUGH_FILTER_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_PASSTHROUGH_FILTER_H

#include <logger/logger.h>

#include <limits>
#include <memory>

#include "flowpipe/stage.h"
#include "flowpipe/stage_registration.h"
#include "height_mapping/core/algorithms/passthrough_filter.h"
#include "height_mapping/pipeline/mapping_context.h"

namespace height_mapping::stages {

/**
 * @brief Pipeline stage adapter for passthrough filter (header-only)
 */
class PassthroughFilter : public flowpipe::Stage {
 public:
  PassthroughFilter()
      : Stage("PassthroughFilter"),
        filter_(std::make_unique<algorithms::PassthroughFilter>()) {}

  void configure(const std::map<std::string, std::string> &params) override {
    algorithms::PassthroughFilter::Config config;

    // Parse axis-aligned bounds
    loadParam(params, "x_min", config.x_min);
    loadParam(params, "x_max", config.x_max);
    loadParam(params, "y_min", config.y_min);
    loadParam(params, "y_max", config.y_max);
    loadParam(params, "z_min", config.z_min);
    loadParam(params, "z_max", config.z_max);

    // Parse distance bounds
    if (loadParam(params, "distance_min", config.distance_min)) {
      config.use_distance_filter = true;
    }
    if (loadParam(params, "distance_max", config.distance_max)) {
      config.use_distance_filter = true;
    }

    // Update or create filter with new config
    if (filter_) {
      filter_->setConfig(config);
    } else {
      filter_ = std::make_unique<algorithms::PassthroughFilter>(config);
    }

    LOG_DEBUG(getName(), "Configured with bounds [", config.x_min, ",",
              config.x_max, "] x [", config.y_min, ",", config.y_max, "] x [",
              config.z_min, ",", config.z_max, "]");
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

    LOG_DEBUG_THROTTLE(1.0, getName(), "PassthroughFilter removed ",
                       stats.removed_count, " points (", stats.original_size,
                       " -> ", stats.filtered_size, ")");
  }

 private:
  std::unique_ptr<algorithms::PassthroughFilter> filter_;
};

}  // namespace height_mapping::stages
#endif  // HEIGHT_MAPPING_PIPELINE_STAGES_PASSTHROUGH_FILTER_H