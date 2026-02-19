// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * kalman_estimation.hpp
 *
 * Height estimation using 1D Kalman filter with Welford sample variance.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_MAPPING_KALMAN_ESTIMATION_HPP
#define FASTDEM_MAPPING_KALMAN_ESTIMATION_HPP

#include <algorithm>
#include <cmath>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

/**
 * @brief Elevation updater using 1D Kalman filter.
 *
 * Uses a simple 1D Kalman filter for height estimation with uncertainty
 * tracking. Measurement variance is provided by sensor model at update time.
 * A parallel Welford accumulator tracks raw measurement spread for the
 * variance output layer and confidence bounds.
 *
 * For static terrain mapping, process noise is omitted since terrain doesn't
 * change over time. Variance bounds prevent numerical issues.
 *
 * Layers created:
 * - elevation: Kalman state estimate (x̂)
 * - elevation_min/max: Observed height range
 * - variance: Sample variance of measurements (Welford)
 * - sample_count: Number of measurements
 * - upper_bound, lower_bound: elevation ± 2√(sample_variance)
 * - uncertainty_range: upper_bound - lower_bound
 *
 * Internal layers (not for visualization/post-processing):
 * - kalman_p: Filter covariance P
 * - sample_mean: Welford running mean (for sample_variance computation)
 */
class KalmanEstimation {
 public:
  KalmanEstimation() = default;

  /**
   * @brief Construct with parameters.
   *
   * @param min_variance Minimum Kalman variance (prevents over-confidence)
   * @param max_variance Maximum Kalman variance (caps uncertainty)
   * @param process_noise Process noise Q (maintains filter receptivity)
   */
  KalmanEstimation(float min_variance, float max_variance, float process_noise)
      : min_variance_(min_variance),
        max_variance_(max_variance),
        process_noise_(process_noise) {}

  /**
   * @brief Initialize layers on the height map.
   *
   * Creates all layers eagerly and caches matrix pointers.
   */
  void initialize(ElevationMap& map) {
    // Common output layers
    if (!map.exists(layer::elevation)) map.add(layer::elevation, NAN);
    if (!map.exists(layer::elevation_min)) map.add(layer::elevation_min, NAN);
    if (!map.exists(layer::elevation_max)) map.add(layer::elevation_max, NAN);
    if (!map.exists(layer::variance)) map.add(layer::variance, 0.0f);
    if (!map.exists(layer::sample_count)) map.add(layer::sample_count, 0.0f);

    // Kalman-internal layers
    if (!map.exists(layer::kalman_p)) map.add(layer::kalman_p, 0.0f);
    if (!map.exists(layer::sample_mean)) map.add(layer::sample_mean, NAN);

    // Derived layers (eagerly created for finalize)
    if (!map.exists(layer::upper_bound)) map.add(layer::upper_bound, NAN);
    if (!map.exists(layer::lower_bound)) map.add(layer::lower_bound, NAN);
    if (!map.exists(layer::uncertainty_range))
      map.add(layer::uncertainty_range, NAN);

    // Cache all matrix pointers
    elevation_mat_ = &map.get(layer::elevation);
    min_mat_ = &map.get(layer::elevation_min);
    max_mat_ = &map.get(layer::elevation_max);
    variance_mat_ = &map.get(layer::variance);
    count_mat_ = &map.get(layer::sample_count);
    kalman_p_mat_ = &map.get(layer::kalman_p);
    sample_mean_mat_ = &map.get(layer::sample_mean);
    upper_mat_ = &map.get(layer::upper_bound);
    lower_mat_ = &map.get(layer::lower_bound);
    range_mat_ = &map.get(layer::uncertainty_range);
  }

  /**
   * @brief Update elevation estimate at a cell using Kalman filter.
   *
   * @param index Pre-computed cell index
   * @param measurement Height measurement value [m]
   * @param measurement_variance Measurement variance from sensor model [m²]
   */
  void update(const grid_map::Index& index,  //
              float measurement,             //
              float measurement_variance) {
    const int i = index(0);
    const int j = index(1);

    // Kalman state: x = elevation, P = kalman_p
    float& x = (*elevation_mat_)(i, j);
    float& P = (*kalman_p_mat_)(i, j);
    float& min_z = (*min_mat_)(i, j);
    float& max_z = (*max_mat_)(i, j);
    float& count = (*count_mat_)(i, j);

    // Welford references
    float& sample_mean = (*sample_mean_mat_)(i, j);
    float& sample_var = (*variance_mat_)(i, j);

    // Measurement (Kalman notation: z = measurement, R = measurement variance)
    const float z = measurement;
    const float R =
        (measurement_variance > 0.0f) ? measurement_variance : max_variance_;

    // Kalman filter update
    if (std::isnan(x)) {
      // First measurement - initialize with measurement variance from sensor
      // model
      x = z;
      P = R;  // Use sensor model uncertainty, not fixed initial_variance
      count = 1.0f;
    } else {
      // Prediction step: add process noise to maintain filter receptivity
      // This prevents over-confidence and absorbs SLAM drift
      P += process_noise_;

      const float K = P / (P + R);  // Kalman gain
      x = x + K * (z - x);          // State update
      P = (1.0f - K) * P;           // Covariance update

      // Clamp covariance to valid range
      P = std::clamp(P, min_variance_, max_variance_);
      count += 1.0f;
    }

    // Welford's online algorithm for sample variance
    if (std::isnan(sample_mean)) {
      sample_mean = z;
      sample_var = 0.0f;
    } else {
      const float delta = z - sample_mean;
      const float new_mean = sample_mean + (delta / count);
      const float delta2 = z - new_mean;

      if (count <= 2.0f) {
        sample_var = delta * delta2;
      } else {
        float m2 = sample_var * (count - 2.0f);
        m2 += delta * delta2;
        sample_var = m2 / (count - 1.0f);
      }

      sample_mean = new_mean;
    }

    // Min/Max update
    if (std::isnan(min_z) || z < min_z) min_z = z;
    if (std::isnan(max_z) || z > max_z) max_z = z;
  }

  /**
   * @brief Compute derived statistics after all updates.
   *
   * Computes confidence bounds from sample variance.
   */
  void finalize() {
    // σ = sqrt(sample variance)
    Eigen::ArrayXXf sigma = variance_mat_->array().sqrt();

    // Confidence bounds: elevation ± 2σ
    *upper_mat_ = elevation_mat_->array() + 2.0f * sigma;
    *lower_mat_ = elevation_mat_->array() - 2.0f * sigma;

    // Uncertainty range = upper - lower
    *range_mat_ = upper_mat_->array() - lower_mat_->array();
  }

  std::string name() const { return "KalmanEstimation"; }

 private:
  float min_variance_ = 0.0001f;
  float max_variance_ = 0.01f;
  float process_noise_ = 0.0f;

  // Common output layer matrices
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* min_mat_ = nullptr;
  grid_map::Matrix* max_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;

  // Kalman-internal layer matrices
  grid_map::Matrix* kalman_p_mat_ = nullptr;
  grid_map::Matrix* sample_mean_mat_ = nullptr;

  // Derived layer matrices
  grid_map::Matrix* upper_mat_ = nullptr;
  grid_map::Matrix* lower_mat_ = nullptr;
  grid_map::Matrix* range_mat_ = nullptr;
};

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_KALMAN_ESTIMATION_HPP
