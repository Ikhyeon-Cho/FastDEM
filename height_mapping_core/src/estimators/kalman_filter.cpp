/*
 * kalman_filter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/estimators/kalman_filter.h"
#include <cmath>

namespace height_mapping::core::estimators {

void KalmanFilter::update(float &elevation, float &variance, float &count,
                          float new_measurement) {
  // First measurement - initialize state
  if (std::isnan(elevation)) {
    elevation = new_measurement;
    variance = params_.initial_variance;
    count = 1.0f;
    return;
  }

  count += 1.0f;

  // Prediction step (no motion model, so state stays the same)
  // P = P + Q
  variance += params_.process_noise;

  // Update step
  // K = P / (P + R)
  const float kalman_gain = variance / (variance + params_.measurement_noise);

  // x = x + K * (z - x)
  elevation = elevation + kalman_gain * (new_measurement - elevation);

  // P = (1 - K) * P
  variance = (1.0f - kalman_gain) * variance;

  // Prevent variance from becoming too small (maintain minimum adaptability)
  const float min_variance = params_.process_noise * 2.0f;
  variance = std::max(variance, min_variance);
}

} // namespace height_mapping::core::estimators