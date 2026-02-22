// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * kalman_estimation.hpp
 *
 * A simple 1D Kalman filter for height estimation with uncertainty tracking.
 *
 * Measurement variance is provided by sensor model at update time.
 * A parallel Welford accumulator tracks raw measurement spread for the
 * variance output layer and confidence bounds.
 * For static terrain mapping, process noise is omitted since terrain doesn't
 * change over time. Variance bounds prevent numerical issues.
 */

#ifndef FASTDEM_MAPPING_KALMAN_ESTIMATION_HPP
#define FASTDEM_MAPPING_KALMAN_ESTIMATION_HPP

#include <algorithm>
#include <cassert>
#include <cmath>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

namespace layer {
constexpr auto kalman_p = "_kalman_p";
constexpr auto sample_mean = "_sample_mean";
constexpr auto sample_m2 = "_sample_m2";
}  // namespace layer

/**
 * @brief Elevation updater using 1D Kalman filter.
 *
 * Layers created:
 * - elevation: Kalman state estimate (x̂)
 * - variance: Sample variance of measurements (Welford)
 * - n_points: Number of measurements
 * - upper_bound, lower_bound: elevation ± 2√(sample_variance)
 *
 * Internal layers (not for visualization/post-processing):
 * - kalman_p: Filter covariance P
 * - sample_mean: Welford running mean (for sample_variance computation)
 * - sample_m2: Welford M2 accumulator (sum of squared deviations)
 */
class Kalman {
 public:
  Kalman() = default;

  /**
   * @brief Construct with parameters.
   *
   * @param min_variance Minimum Kalman variance (prevents over-confidence)
   * @param max_variance Maximum Kalman variance (caps uncertainty)
   * @param process_noise Process noise Q (maintains filter receptivity)
   */
  Kalman(float min_variance, float max_variance, float process_noise)
      : min_variance_(min_variance),
        max_variance_(max_variance),
        process_noise_(process_noise) {}

  /// Create required layers on the map. Call once before first update.
  void ensureLayers(ElevationMap& map) {
    // clang-format off
    if (!map.exists(layer::variance))
      map.add(layer::variance, 0.0f);
    if (!map.exists(layer::n_points))
      map.add(layer::n_points, 0.0f);
    if (!map.exists(layer::kalman_p))
      map.add(layer::kalman_p, 0.0f);
    if (!map.exists(layer::sample_mean))
      map.add(layer::sample_mean, NAN);
    if (!map.exists(layer::sample_m2))
      map.add(layer::sample_m2, 0.0f);

    if (!map.exists(layer::upper_bound))
      map.add(layer::upper_bound, NAN);
    if (!map.exists(layer::lower_bound))
      map.add(layer::lower_bound, NAN);
    // clang-format on
  }

  /// Cache matrix pointers. Call each frame before update loop.
  void bind(ElevationMap& map) {
    elevation_mat_ = &map.get(layer::elevation);
    variance_mat_ = &map.get(layer::variance);
    count_mat_ = &map.get(layer::n_points);
    kalman_p_mat_ = &map.get(layer::kalman_p);
    sample_mean_mat_ = &map.get(layer::sample_mean);
    sample_m2_mat_ = &map.get(layer::sample_m2);
    upper_mat_ = &map.get(layer::upper_bound);
    lower_mat_ = &map.get(layer::lower_bound);
    bound_ = true;
  }

  /// Update elevation estimate at a single cell.
  void update(const grid_map::Index& index, float measurement,
              float measurement_variance) {
    assert(bound_ && "Kalman::bind() must be called before update()");
    const int i = index(0);
    const int j = index(1);

    float& x = (*elevation_mat_)(i, j);
    float& P = (*kalman_p_mat_)(i, j);
    float& count = (*count_mat_)(i, j);
    float& sample_mean = (*sample_mean_mat_)(i, j);
    float& sample_var = (*variance_mat_)(i, j);
    float& m2 = (*sample_m2_mat_)(i, j);

    const float z = measurement;
    const float R =
        (measurement_variance > 0.0f) ? measurement_variance : max_variance_;

    // Kalman filter
    if (std::isnan(x)) {
      x = z;
      P = R;
      count = 1.0f;
    } else {
      P += process_noise_;
      const float K = P / (P + R);
      x = x + K * (z - x);
      P = (1.0f - K) * P;
      P = std::clamp(P, min_variance_, max_variance_);
      count += 1.0f;
    }

    // Welford's online variance
    if (std::isnan(sample_mean)) {
      sample_mean = z;
      sample_var = 0.0f;
      m2 = 0.0f;
    } else {
      const float delta = z - sample_mean;
      const float new_mean = sample_mean + (delta / count);
      const float delta2 = z - new_mean;
      m2 += delta * delta2;
      sample_var = (count > 1.0f) ? m2 / (count - 1.0f) : 0.0f;
      sample_mean = new_mean;
    }
  }

  /// Compute confidence bounds from accumulated statistics.
  void computeBounds() {
    assert(bound_ && "Kalman::bind() must be called before computeBounds()");
    // σ = sqrt(sample variance)
    Eigen::ArrayXXf sigma = variance_mat_->array().max(0.0f).sqrt();

    // Confidence bounds: elevation ± 2σ
    *upper_mat_ = elevation_mat_->array() + 2.0f * sigma;
    *lower_mat_ = elevation_mat_->array() - 2.0f * sigma;
  }

 private:
  float min_variance_ = 0.0001f;
  float max_variance_ = 0.01f;
  float process_noise_ = 0.0f;

  // Common output layer matrices
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;

  // Kalman-internal layer matrices
  grid_map::Matrix* kalman_p_mat_ = nullptr;
  grid_map::Matrix* sample_mean_mat_ = nullptr;
  grid_map::Matrix* sample_m2_mat_ = nullptr;

  // Derived layer matrices
  grid_map::Matrix* upper_mat_ = nullptr;
  grid_map::Matrix* lower_mat_ = nullptr;

  bool bound_ = false;
};

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_KALMAN_ESTIMATION_HPP
