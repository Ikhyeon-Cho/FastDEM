/*
 * estimator_factory.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/estimators/estimator_factory.h"
#include "height_mapping_core/estimators/incremental_mean.h"
#include "height_mapping_core/estimators/kalman_filter.h"
#include "height_mapping_core/estimators/moving_average.h"
#include <algorithm>
#include <logger/logger.h>

namespace height_mapping::core::estimators {

std::unique_ptr<HeightEstimatorBase>
EstimatorFactory::create(const std::string &type,
                         const std::map<std::string, std::string> &params) {

  // Normalize type string to lowercase for case-insensitive comparison
  std::string lower_type = type;
  std::transform(lower_type.begin(), lower_type.end(), lower_type.begin(),
                 ::tolower);

  // Kalman Filter
  if (lower_type == "kalman_filter") {
    KalmanFilter::Parameters kalman_params;
    kalman_params.process_noise = getFloatParam(params, "process_noise", 0.01f);
    kalman_params.measurement_noise =
        getFloatParam(params, "measurement_noise", 0.1f);
    kalman_params.initial_variance =
        getFloatParam(params, "initial_variance", 1.0f);

    return std::make_unique<KalmanFilter>(kalman_params);
  }

  // Incremental Mean
  if (lower_type == "incremental_mean") {
    return std::make_unique<IncrementalMean>();
  }

  // Moving Average
  if (lower_type == "moving_average") {
    MovingAverage::Parameters ma_params;
    ma_params.alpha = getFloatParam(params, "alpha", 0.3f);

    // Validate alpha is in [0, 1]
    if (ma_params.alpha < 0.0f || ma_params.alpha > 1.0f) {
      LOG_WARN("EstimatorFactory", "Alpha=", ma_params.alpha,
               " out of range [0,1], clamping");
      ma_params.alpha = std::max(0.01f, std::min(0.99f, ma_params.alpha));
    }

    return std::make_unique<MovingAverage>(ma_params);
  }

  // Unknown type - default to IncrementalMean
  LOG_ERROR("EstimatorFactory", "Unknown estimator type: '", type,
            "', using IncrementalMean as default");
  return std::make_unique<IncrementalMean>();
}

float EstimatorFactory::getFloatParam(
    const std::map<std::string, std::string> &params,
    const std::string &key, float defaultValue) {

  auto it = params.find(key);
  if (it == params.end()) {
    return defaultValue;
  }

  try {
    return std::stof(it->second);
  } catch (const std::exception &e) {
    LOG_WARN("EstimatorFactory", "Failed to parse '", key, "' as float: ",
             it->second, ", using default: ", defaultValue);
    return defaultValue;
  }
}

} // namespace height_mapping::core::estimators