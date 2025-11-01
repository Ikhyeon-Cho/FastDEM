/*
 * statistical_ground_segmentation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_STATISTICAL_GROUND_SEGMENTATION_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_STATISTICAL_GROUND_SEGMENTATION_H

#include "height_mapping_core/algorithms/statistical_ground_segmentation.h"
#include "height_mapping_pipeline/mapping_context.h"
#include "flowpipe/stage.h"
#include "flowpipe/stage_registration.h"
#include <logger/logger.h>
#include <memory>

namespace height_mapping::mapping::stages {

/**
 * @brief Pipeline stage adapter for statistical ground segmentation
 * (header-only)
 *
 * This stage wraps the core StatisticalGroundSegmentation algorithm for use in
 * the pipeline. It handles configuration parsing from YAML and context
 * management.
 */
class StatisticalGroundSegmentation : public flowpipe::Stage {
public:
  StatisticalGroundSegmentation()
      : Stage("StatisticalGroundSegmentation"),
        segmenter_(
            std::make_unique<algorithms::StatisticalGroundSegmentation>()) {}

  void configure(const std::map<std::string, std::string> &params) override {
    algorithms::StatisticalGroundSegmentation::Config config;

    // Parse parameters - only update if present in params
    loadParam(params, "ground_percentile", config.ground_percentile);
    loadParam(params, "ground_thickness", config.ground_thickness);
    loadParam(params, "noise_threshold", config.noise_threshold);
    loadParam(params, "keep_only_ground", config.keep_only_ground);

    // Update or create segmenter with new config
    if (segmenter_) {
      segmenter_->setConfig(config);
    } else {
      segmenter_ =
          std::make_unique<algorithms::StatisticalGroundSegmentation>(config);
    }

    LOG_DEBUG(getName(),
              "Robust ground extraction: percentile=", config.ground_percentile,
              ", thickness=", config.ground_thickness, "m");
  }

protected:
  void processImpl(flowpipe::Context &ctx) override {
    auto &mapping_ctx = static_cast<MappingContext &>(ctx);
    auto &cloud = mapping_ctx.cloud();

    if (cloud.empty()) {
      LOG_WARN(getName(), "Input cloud is empty, skipping segmentation...");
      return;
    }

    // Run the core algorithm in-place
    auto stats = segmenter_->segmentInPlace(cloud);

    // Log statistics
    LOG_DEBUG_THROTTLE(
        1.0, getName(), "Robust ground extraction: ", stats.ground_count,
        " ground, ", stats.obstacle_count, " obstacle, ", stats.noise_count,
        " noise points (",
        static_cast<int>(segmenter_->getConfig().ground_percentile * 100),
        "th percentile z=", stats.robust_min_z,
        "m, threshold=", stats.ground_threshold, "m)");
  }

private:
  std::unique_ptr<algorithms::StatisticalGroundSegmentation> segmenter_;
};

REGISTER_STAGE(StatisticalGroundSegmentation)

} // namespace height_mapping::mapping::stages
#endif // HEIGHT_MAPPING_PIPELINE_STAGES_STATISTICAL_GROUND_SEGMENTATION_H