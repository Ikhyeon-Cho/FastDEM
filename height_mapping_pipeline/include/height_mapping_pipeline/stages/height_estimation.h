/*
 * height_estimation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_STAGES_HEIGHT_ESTIMATION_H
#define HEIGHT_MAPPING_PIPELINE_STAGES_HEIGHT_ESTIMATION_H

#include "height_mapping_core/algorithms/height_estimator.h"
#include "height_mapping_pipeline/mapping_context.h"
#include "pipeline_core/stage.h"
#include "pipeline_core/stage_registration.h"
#include <logger/logger.h>
#include <memory>

namespace height_mapping::mapping::stages {

/**
 * @brief Pipeline stage adapter for height estimation (header-only)
 */
class HeightEstimation : public pipeline::Stage {
public:
  HeightEstimation()
      : Stage("HeightEstimation"),
        estimator_(std::make_unique<algorithms::HeightEstimator>()) {}

  void configure(const std::map<std::string, std::string> &params) override {
    algorithms::HeightEstimator::Config config;

    // Load estimator type
    std::string estimator_type_str;
    if (loadParam(params, "estimator_type", estimator_type_str)) {
      if (estimator_type_str == "kalman_filter") {
        config.estimator_type =
            algorithms::HeightEstimator::Config::EstimatorType::KALMAN_FILTER;
      } else if (estimator_type_str == "moving_average") {
        config.estimator_type =
            algorithms::HeightEstimator::Config::EstimatorType::MOVING_AVERAGE;
      } else if (estimator_type_str == "incremental_mean") {
        config.estimator_type = algorithms::HeightEstimator::Config::
            EstimatorType::INCREMENTAL_MEAN;
      } else {
        LOG_ERROR(getName(), "Unknown estimator type: ", estimator_type_str,
                  ", using incremental_mean");
        config.estimator_type = algorithms::HeightEstimator::Config::
            EstimatorType::INCREMENTAL_MEAN;
      }
    }

    // Load Kalman parameters
    loadParam(params, "kalman_process_noise", config.kalman_process_noise);
    loadParam(params, "kalman_measurement_noise",
              config.kalman_measurement_noise);
    loadParam(params, "kalman_initial_variance",
              config.kalman_initial_variance);

    // Load Moving Average parameters
    loadParam(params, "moving_average_alpha", config.moving_average_alpha);

    // Update or create estimator with new config
    if (estimator_) {
      estimator_->setConfig(config);
    } else {
      estimator_ = std::make_unique<algorithms::HeightEstimator>(config);
    }

    std::string estimator_name = "incremental_mean";
    switch (config.estimator_type) {
    case algorithms::HeightEstimator::Config::EstimatorType::KALMAN_FILTER:
      estimator_name = "kalman_filter";
      break;
    case algorithms::HeightEstimator::Config::EstimatorType::MOVING_AVERAGE:
      estimator_name = "moving_average";
      break;
    default:
      break;
    }
    LOG_DEBUG(getName(), "Configured with ", estimator_name, " estimator");
  }

protected:
  void processImpl(pipeline::Context &ctx) override {

    auto &mapping_ctx = static_cast<MappingContext &>(ctx);
    auto &map = mapping_ctx.map();
    auto &cloud = mapping_ctx.cloud();

    if (cloud.empty()) {
      return;
    }

    // Run the core algorithm
    auto stats = estimator_->updateMap(map, cloud);

    LOG_DEBUG_THROTTLE(1.0, getName(),
                       "Updated height map: ", stats.cells_updated,
                       " cells from ", stats.points_processed, " points (",
                       stats.points_out_of_bounds, " out of bounds)");
  }

private:
  std::unique_ptr<algorithms::HeightEstimator> estimator_;
};

REGISTER_STAGE(HeightEstimation)

} // namespace height_mapping::mapping::stages
#endif // HEIGHT_MAPPING_PIPELINE_STAGES_HEIGHT_ESTIMATION_H