/*
 * kalman_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ESTIMATORS_KALMAN_FILTER_H
#define HEIGHT_MAPPING_CORE_ESTIMATORS_KALMAN_FILTER_H

#include "height_mapping/core/estimators/height_estimator_base.h"

namespace height_mapping::estimators {

/**
 * @brief Kalman filter estimator for height measurements
 *
 * Uses a simple 1D Kalman filter to estimate height with uncertainty tracking.
 */
class KalmanFilter : public HeightEstimatorBase {
 public:
  struct Parameters {
    float process_noise{0.01f};     // Q: Process noise covariance
    float measurement_noise{0.1f};  // R: Measurement noise covariance
    float initial_variance{1.0f};   // P0: Initial state covariance
  };

  KalmanFilter() : params_() {}
  explicit KalmanFilter(const Parameters &params) : params_(params) {}

  void update(float &elevation, float &variance, float &count,
              float newMeasurement) override;

  std::string name() const override { return "KalmanFilter"; }
  void setParameters(const Parameters &params) { params_ = params; }

 private:
  Parameters params_;
};

}  // namespace height_mapping::estimators

#endif  // HEIGHT_MAPPING_CORE_ESTIMATORS_KALMAN_FILTER_H