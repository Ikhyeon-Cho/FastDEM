/*
 * transform_cloud.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_TRANSFORM_CLOUD_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_TRANSFORM_CLOUD_H

#include "height_mapping_core/algorithms/transform_cloud.h"
#include "height_mapping_pipeline/interfaces/transform_provider.h"
#include "height_mapping_pipeline/mapping_context.h"
#include "flowpipe/stage.h"
#include "flowpipe/stage_registration.h"
#include <logger/logger.h>
#include <memory>
#include <stdexcept>

namespace height_mapping::mapping::stages {

/**
 * @brief Pipeline stage adapter for cloud transformation (header-only)
 *
 * This stage requires ITransformProvider service to be available in the
 * context.
 */
class TransformCloud : public flowpipe::Stage {
public:
  TransformCloud()
      : Stage("TransformCloud"),
        transformer_(std::make_unique<algorithms::TransformCloud>()) {}

  void configure(const std::map<std::string, std::string> &params) override {
    algorithms::TransformCloud::Config config;

    // Parse target frame
    loadParam(params, "target_frame", config.target_frame);

    // Parse skip_if_same_frame option
    loadParam(params, "skip_if_same_frame", config.skip_if_same_frame);

    // Update or create transformer with new config
    if (transformer_) {
      transformer_->setConfig(config);
    } else {
      transformer_ = std::make_unique<algorithms::TransformCloud>(config);
    }

    LOG_DEBUG(getName(), "Configured with target_frame=", config.target_frame);
  }

protected:
  void processImpl(flowpipe::Context &ctx) override {
    auto &mapping_ctx = static_cast<MappingContext &>(ctx);
    auto &cloud = mapping_ctx.cloud();

    // Check if transformation is needed
    if (!transformer_->needsTransform(cloud)) {
      return;
    }

    // Get transform provider from context (required)
    auto transform_provider = ctx.getService<mapping::ITransformProvider>();
    if (!transform_provider) {
      throw std::runtime_error("TransformCloud stage requires a transform "
                               "provider to be set in context");
    }

    // Lookup transform
    auto transform_opt = transform_provider->lookupTransform(
        transformer_->getConfig().target_frame, cloud.frameId(),
        cloud.timestamp());

    if (!transform_opt.has_value()) {
      throw std::runtime_error("Failed to get transform from '" +
                               cloud.frameId() + "' to '" +
                               transformer_->getConfig().target_frame + "'");
    }

    // Apply transform in-place
    auto stats = transformer_->transformInPlace(cloud, transform_opt.value());

    if (stats.transform_applied) {
      LOG_DEBUG_THROTTLE(1.0, getName(), "Transformed ",
                         stats.points_transformed, " points from ",
                         stats.original_frame, " to ", stats.target_frame);
    }
  }

private:
  std::unique_ptr<algorithms::TransformCloud> transformer_;
};

REGISTER_STAGE(TransformCloud)

} // namespace height_mapping::mapping::stages
#endif // HEIGHT_MAPPING_PIPELINE_STAGES_TRANSFORM_CLOUD_H