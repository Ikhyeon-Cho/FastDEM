/*
 * height_estimation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_HEIGHT_ESTIMATION_H
#define HEIGHT_MAPPING_CORE_STAGES_HEIGHT_ESTIMATION_H

#include "pipeline_core/stage.h"
#include "height_mapping_core/estimators/height_estimator_base.h"
#include <memory>

namespace height_mapping::core::stages {

/**
 * @brief Stage that estimates terrain height from point clouds
 *
 * Supports multiple estimation methods: Kalman filter, Mean, Moving Average
 */
class HeightEstimation : public pipeline::Stage {
public:
  HeightEstimation();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "estimator_type": "incremental_mean", "kalman_filter", or "moving_average"
   *        - "process_noise": Process noise for Kalman filter (default: 0.01)
   *        - "measurement_noise": Measurement noise for Kalman filter (default: 0.1)
   *        - "alpha": Weight for moving average [0,1] (default: 0.3)
   */
  void configure(const std::map<std::string, std::string> &params) override;

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  std::unique_ptr<estimators::HeightEstimatorBase> estimator_;
  std::string estimator_type_ = "mean";
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_HEIGHT_ESTIMATION_H