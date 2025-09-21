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
#include <memory>

namespace height_mapping::core {

/**
 * @brief Stage that estimates terrain height from point clouds
 *
 * Supports multiple estimation methods: Kalman filter, Mean, Moving Average
 */
class HeightEstimationStage : public pipeline::Stage {
public:
  enum class EstimatorType {
    KALMAN,
    INCREMENTAL_MEAN,
    MOVING_AVERAGE
  };

  HeightEstimationStage();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "estimator_type": kalman, mean, or moving_average
   *        - "process_noise": Process noise for Kalman filter
   *        - "measurement_noise": Measurement noise for Kalman filter
   *        - "window_size": Window size for moving average
   */
  void configure(const std::map<std::string, std::string>& params) override;

  void setEstimatorType(EstimatorType type) { estimator_type_ = type; }

protected:
  void processImpl(pipeline::Context& ctx) override;

private:
  EstimatorType estimator_type_ = EstimatorType::KALMAN;

  // Kalman parameters
  float process_noise_ = 0.01f;
  float measurement_noise_ = 0.1f;

  // Moving average parameters
  int window_size_ = 10;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_STAGES_HEIGHT_ESTIMATION_H