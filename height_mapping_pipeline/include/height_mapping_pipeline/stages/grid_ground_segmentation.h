/*
 * grid_ground_segmentation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_GRID_GROUND_SEGMENTATION_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_GRID_GROUND_SEGMENTATION_H

#include "height_mapping_core/algorithms/grid_ground_segmentation.h"
#include "height_mapping_pipeline/mapping_context.h"
#include "pipeline_core/stage.h"
#include "pipeline_core/stage_registration.h"
#include <logger/logger.h>
#include <memory>

namespace height_mapping::mapping::stages {

/**
 * @brief Pipeline stage adapter for grid-based ground segmentation
 * (header-only)
 *
 * This stage wraps the core GridGroundSegmentation algorithm for use in the
 * pipeline. It handles configuration parsing from YAML and context management.
 */
class GridGroundSegmentation : public pipeline::Stage {
public:
  GridGroundSegmentation()
      : Stage("GridGroundSegmentation"),
        segmenter_(std::make_unique<algorithms::GridGroundSegmentation>()) {}

  void configure(const std::map<std::string, std::string> &params) override {
    algorithms::GridGroundSegmentation::Config config;

    // Parse parameters - only update if present in params
    loadParam(params, "grid_resolution", config.grid_resolution);
    loadParam(params, "cell_percentile", config.cell_percentile);
    loadParam(params, "ground_thickness", config.ground_thickness);
    loadParam(params, "max_ground_height", config.max_ground_height);
    loadParam(params, "min_points_per_cell", config.min_points_per_cell);
    loadParam(params, "keep_only_ground", config.keep_only_ground);

    // Update or create segmenter with new config
    if (segmenter_) {
      segmenter_->setConfig(config);
    } else {
      segmenter_ = std::make_unique<algorithms::GridGroundSegmentation>(config);
    }

    LOG_DEBUG(getName(),
              "Configured with grid_resolution=", config.grid_resolution,
              "m, cell_percentile=", config.cell_percentile,
              ", ground_thickness=", config.ground_thickness, "m");
  }

protected:
  void processImpl(pipeline::Context &ctx) override {

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
        1.0, getName(), "Grid ground segmentation: ", stats.total_cells,
        " total cells, ", stats.processed_cells, " processed, ",
        stats.obstacle_only_cells, " obstacle-only cells; ", stats.ground_count,
        " ground points, ", stats.obstacle_count, " obstacle points, ",
        stats.unprocessed_count, " unprocessed points");
  }

private:
  std::unique_ptr<algorithms::GridGroundSegmentation> segmenter_;
};

REGISTER_STAGE(GridGroundSegmentation)

} // namespace height_mapping::mapping::stages
#endif // HEIGHT_MAPPING_PIPELINE_STAGES_GRID_GROUND_SEGMENTATION_H