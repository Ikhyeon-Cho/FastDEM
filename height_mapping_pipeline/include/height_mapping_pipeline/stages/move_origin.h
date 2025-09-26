/*
 * move_origin.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_MOVE_ORIGIN_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_MOVE_ORIGIN_H

#include "height_mapping_core/algorithms/move_origin.h"
#include "height_mapping_pipeline/interfaces/transform_provider.h"
#include "height_mapping_pipeline/mapping_context.h"
#include "pipeline_core/stage.h"
#include "pipeline_core/stage_registration.h"
#include <logger/logger.h>
#include <memory>

namespace height_mapping::mapping::stages {

/**
 * @brief Pipeline stage adapter for map origin updates (header-only)
 *
 * This stage requires ITransformProvider service to get robot position.
 */
class MoveOrigin : public pipeline::Stage {
public:
  MoveOrigin()
      : Stage("MoveOrigin"),
        updater_(std::make_unique<algorithms::MoveOrigin>()) {}

  void configure(const std::map<std::string, std::string> &params) override {
    algorithms::MoveOrigin::Config config;

    // Parse update mode
    std::string mode;
    if (loadParam(params, "update_mode", mode)) {
      if (mode == "continuous") {
        config.update_mode =
            algorithms::MoveOrigin::UpdateMode::CONTINUOUS;
      } else if (mode == "threshold") {
        config.update_mode =
            algorithms::MoveOrigin::UpdateMode::THRESHOLD;
      } else if (mode == "centered") {
        config.update_mode = algorithms::MoveOrigin::UpdateMode::CENTERED;
      } else if (mode == "fixed") {
        config.update_mode = algorithms::MoveOrigin::UpdateMode::FIXED;
      } else {
        LOG_ERROR(getName(), "Unknown update mode: ", mode,
                  ", using CONTINUOUS");
      }
    }

    // Parse other parameters
    loadParam(params, "update_threshold", config.update_threshold);
    loadParam(params, "center_region_size", config.center_region_size);
    loadParam(params, "robot_frame", config.robot_frame);
    loadParam(params, "map_frame", config.map_frame);

    // Update or create updater with new config
    if (updater_) {
      updater_->setConfig(config);
    } else {
      updater_ = std::make_unique<algorithms::MoveOrigin>(config);
    }

    LOG_DEBUG(getName(), "Configured with update_mode=",
              params.count("update_mode") ? params.at("update_mode")
                                          : "continuous");
  }

protected:
  void processImpl(pipeline::Context &ctx) override {
    auto &mapping_ctx = static_cast<MappingContext &>(ctx);
    auto &map = mapping_ctx.map();
    auto &cloud = mapping_ctx.cloud();

    // Get transform provider to lookup robot position
    auto transform_provider = ctx.getService<mapping::ITransformProvider>();
    if (!transform_provider) {
      // If no transform provider, can't update map origin
      return;
    }

    const auto &config = updater_->getConfig();

    // Get robot position in map frame using cloud timestamp
    // This ensures temporal consistency with the sensor data
    auto transform_opt = transform_provider->lookupTransform(
        config.map_frame, config.robot_frame, cloud.timestamp());

    if (!transform_opt.has_value()) {
      // Can't get robot position, skip update
      return;
    }

    // Extract robot position from transform
    const auto &T = transform_opt.value().matrix();
    Eigen::Vector3f robot_position(T(0, 3), T(1, 3),
                                   0.0f); // Only care about x,y

    // Run the core algorithm
    auto stats = updater_->updateOrigin(map, robot_position);

    if (stats.updated) {
      LOG_DEBUG_THROTTLE(1.0, getName(),
                         "Map origin updated: reason=", stats.update_reason,
                         ", moved ", stats.distance_moved, "m",
                         " (total: ", stats.total_updates, " updates, ",
                         stats.total_distance, "m)");
    }
  }

private:
  std::unique_ptr<algorithms::MoveOrigin> updater_;
};

REGISTER_STAGE(MoveOrigin)

} // namespace height_mapping::mapping::stages
#endif // HEIGHT_MAPPING_PIPELINE_STAGES_MOVE_ORIGIN_H