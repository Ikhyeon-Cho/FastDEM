/*
 * kalman_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ESTIMATOR_KALMAN_FILTER_H
#define HEIGHT_MAPPING_CORE_ESTIMATOR_KALMAN_FILTER_H

#include <algorithm>
#include <cmath>

#include "height_mapping/core/estimator/base.h"

namespace height_mapping::estimator {

/**
 * @brief Kalman filter estimator for height measurements
 *
 * Uses a simple 1D Kalman filter to estimate height with uncertainty tracking.
 */
class KalmanFilter : public Base {
 public:
  KalmanFilter() = default;
  KalmanFilter(float process_noise, float measurement_noise,
               float initial_variance)
      : process_noise_(process_noise),
        measurement_noise_(measurement_noise),
        initial_variance_(initial_variance) {}

  void update(float& elevation, float& variance, float& count,
              float new_measurement) override {
    // First measurement - initialize state
    if (std::isnan(elevation)) {
      elevation = new_measurement;
      variance = initial_variance_;
      count = 1.0f;
      return;
    }

    count += 1.0f;

    // Prediction step: P = P + Q
    variance += process_noise_;

    // Update step: K = P / (P + R)
    const float kalman_gain = variance / (variance + measurement_noise_);

    // x = x + K * (z - x)
    elevation = elevation + kalman_gain * (new_measurement - elevation);

    // P = (1 - K) * P
    variance = (1.0f - kalman_gain) * variance;

    // Prevent variance from becoming too small
    const float min_variance = process_noise_ * 2.0f;
    variance = std::max(variance, min_variance);
  }

  std::string name() const override { return "KalmanFilter"; }

 private:
  float process_noise_ = 0.01f;
  float measurement_noise_ = 0.1f;
  float initial_variance_ = 1.0f;
};

}  // namespace height_mapping::estimator

#endif  // HEIGHT_MAPPING_CORE_ESTIMATOR_KALMAN_FILTER_H
