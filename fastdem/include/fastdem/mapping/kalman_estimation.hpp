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
 * A parallel Welford accumulator tracks raw measurement spread for
 * upper-bound elevation output: elevation = state + k * sqrt(sample_variance).
 *
 * For static terrain mapping, process noise is omitted since terrain doesn't
 * change over time. Variance bounds prevent numerical issues.
 *
 * Layers created:
 * - elevation: Height estimate (state + k * σ_sample)
 * - state: Kalman state estimate (x)
 * - elevation_min: Minimum observed height
 * - elevation_max: Maximum observed height
 * - variance: Kalman covariance (P)
 * - mean: Running mean of raw measurements (Welford)
 * - sample_variance: Sample variance of raw measurements (Welford)
 * - upper_bound, lower_bound: Confidence bounds (state ± 2σ_kalman)
 * - uncertainty_range: upper_bound - lower_bound
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
   * @param sigma_scale Sigma multiplier for elevation output (0 = state, 1 ≈ 84th percentile)
   */
  KalmanEstimation(float min_variance, float max_variance, float process_noise,
                   float sigma_scale = 0.0f)
      : min_variance_(min_variance),
        max_variance_(max_variance),
        process_noise_(process_noise),
        sigma_scale_(sigma_scale) {}

  /**
   * @brief Initialize layers on the height map.
   *
   * Creates all layers (core + Welford + derived) eagerly and caches matrix
   * pointers.
   */
  void initialize(ElevationMap& map) {
    map_ = &map;

    // Core Kalman layers
    if (!map.exists(layer::elevation)) map.add(layer::elevation, NAN);
    if (!map.exists(layer::elevation_min)) map.add(layer::elevation_min, NAN);
    if (!map.exists(layer::elevation_max)) map.add(layer::elevation_max, NAN);
    if (!map.exists(layer::state)) map.add(layer::state, NAN);
    if (!map.exists(layer::variance)) map.add(layer::variance, 0.0f);
    if (!map.exists(layer::sample_count)) map.add(layer::sample_count, 0.0f);

    // Welford layers for raw measurement statistics
    if (!map.exists(layer::mean)) map.add(layer::mean, NAN);
    if (!map.exists(layer::sample_variance))
      map.add(layer::sample_variance, 0.0f);

    // Derived layers (eagerly created for finalize)
    if (!map.exists(layer::upper_bound)) map.add(layer::upper_bound, NAN);
    if (!map.exists(layer::lower_bound)) map.add(layer::lower_bound, NAN);
    if (!map.exists(layer::uncertainty_range))
      map.add(layer::uncertainty_range, NAN);

    // Cache all matrix pointers
    elevation_mat_ = &map.get(layer::elevation);
    min_mat_ = &map.get(layer::elevation_min);
    max_mat_ = &map.get(layer::elevation_max);
    state_mat_ = &map.get(layer::state);
    variance_mat_ = &map.get(layer::variance);
    count_mat_ = &map.get(layer::sample_count);
    mean_mat_ = &map.get(layer::mean);
    sample_var_mat_ = &map.get(layer::sample_variance);
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

    // Map layer references (Kalman notation: x = state, P = covariance)
    float& x = (*state_mat_)(i, j);
    float& P = (*variance_mat_)(i, j);
    float& min_z = (*min_mat_)(i, j);
    float& max_z = (*max_mat_)(i, j);
    float& count = (*count_mat_)(i, j);

    // Welford references
    float& sample_mean = (*mean_mat_)(i, j);
    float& sample_var = (*sample_var_mat_)(i, j);

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

    // Welford's online algorithm for raw measurement statistics
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

    // Elevation: state + sigma_scale * σ_sample
    float sigma = (sample_var > 0.0f) ? std::sqrt(sample_var) : 0.0f;
    (*elevation_mat_)(i, j) = x + sigma_scale_ * sigma;
  }

  /**
   * @brief Compute derived statistics after all updates.
   *
   * Computes 95% confidence bounds from Kalman covariance.
   */
  void finalize() {
    // σ = sqrt(Kalman variance)
    Eigen::ArrayXXf sigma = variance_mat_->array().sqrt();

    // Confidence bounds: state ± 2σ
    *upper_mat_ = state_mat_->array() + 2.0f * sigma;
    *lower_mat_ = state_mat_->array() - 2.0f * sigma;

    // Uncertainty range = upper - lower
    *range_mat_ = upper_mat_->array() - lower_mat_->array();
  }

  std::string name() const { return "KalmanEstimation"; }

 private:
  float min_variance_ = 0.0001f;
  float max_variance_ = 0.01f;
  float process_noise_ = 0.0f;
  float sigma_scale_ = 0.0f;

  // Map pointer for finalize operations
  ElevationMap* map_ = nullptr;

  // Core Kalman layer matrices
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* min_mat_ = nullptr;
  grid_map::Matrix* max_mat_ = nullptr;
  grid_map::Matrix* state_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;

  // Welford layer matrices
  grid_map::Matrix* mean_mat_ = nullptr;
  grid_map::Matrix* sample_var_mat_ = nullptr;

  // Derived layer matrices
  grid_map::Matrix* upper_mat_ = nullptr;
  grid_map::Matrix* lower_mat_ = nullptr;
  grid_map::Matrix* range_mat_ = nullptr;
};

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_KALMAN_ESTIMATION_HPP
