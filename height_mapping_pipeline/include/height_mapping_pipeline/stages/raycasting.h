/*
 * raycasting.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_RAYCASTING_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_RAYCASTING_H

#include "height_mapping_core/algorithms/raycaster.h"
#include "height_mapping_pipeline/interfaces/transform_provider.h"
#include "height_mapping_pipeline/mapping_context.h"
#include "pipeline_core/stage.h"
#include "pipeline_core/stage_registration.h"
#include <cmath>
#include <logger/logger.h>
#include <memory>

namespace height_mapping::mapping::stages {

/**
 * @brief Pipeline stage adapter for raycasting correction (header-only)
 *
 * This stage optionally uses ITransformProvider service to get sensor origin.
 */
class Raycasting : public pipeline::Stage {
public:
  Raycasting()
      : Stage("Raycasting"),
        raycaster_(std::make_unique<algorithms::Raycaster>()) {}

  void configure(const std::map<std::string, std::string> &params) override {
    algorithms::Raycaster::Config config;

    // Parse max ground angle (convert from degrees to radians)
    float angle_degrees;
    if (loadParam(params, "max_ground_angle", angle_degrees)) {
      config.max_ground_angle = angle_degrees * M_PI / 180.0f;
    }

    // Parse other parameters
    loadParam(params, "correction_threshold", config.correction_threshold);
    loadParam(params, "enable_correction", config.enable_correction);
    loadParam(params, "ray_step_size", config.ray_step_size);

    // Update or create raycaster with new config
    if (raycaster_) {
      raycaster_->setConfig(config);
    } else {
      raycaster_ = std::make_unique<algorithms::Raycaster>(config);
    }

    LOG_DEBUG(getName(), "Configured with max_ground_angle=",
              config.max_ground_angle * 180.0f / M_PI,
              " degrees, threshold=", config.correction_threshold, "m");
  }

protected:
  void processImpl(pipeline::Context &ctx) override {
    auto &mapping_ctx = static_cast<MappingContext &>(ctx);
    auto &map = mapping_ctx.map();
    auto &cloud = mapping_ctx.cloud();

    if (cloud.empty() || !raycaster_->getConfig().enable_correction) {
      return;
    }

    // Get sensor origin from transform provider if available
    Eigen::Vector3f sensor_origin(0, 0, 0);
    auto transform_provider = ctx.getService<mapping::ITransformProvider>();
    if (transform_provider) {
      // Get transform from map to sensor frame to find sensor position in map
      auto transform_opt = transform_provider->lookupTransform(
          cloud.frameId(), "base_link", cloud.timestamp());
      if (transform_opt.has_value()) {
        // Sensor is typically at base_link origin
        sensor_origin = transform_opt.value().translation();
      }
    }

    // Run the core algorithm
    auto stats = raycaster_->correctMap(map, cloud, sensor_origin);

    LOG_DEBUG_THROTTLE(1.0, getName(), "Raycasting: ", stats.total_rays_cast,
                       " rays, ", stats.ground_points_detected,
                       " ground points, ", stats.cells_corrected,
                       " cells corrected");
  }

private:
  std::unique_ptr<algorithms::Raycaster> raycaster_;
};

REGISTER_STAGE(Raycasting)

} // namespace height_mapping::mapping::stages
#endif // HEIGHT_MAPPING_PIPELINE_STAGES_RAYCASTING_H