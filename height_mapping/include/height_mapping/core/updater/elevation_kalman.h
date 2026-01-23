/*
 * elevation_kalman.h
 *
 * Elevation layer updater using 1D Kalman filter.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_KALMAN_H
#define HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_KALMAN_H

#include <algorithm>
#include <cmath>

#include "height_mapping/core/layers.h"
#include "height_mapping/core/map.h"
#include "height_mapping/core/updater/utility.h"

namespace height_mapping::updater {

/**
 * @brief Elevation updater using 1D Kalman filter.
 *
 * Uses a simple 1D Kalman filter for height estimation with uncertainty
 * tracking. Measurement variance is provided by sensor model at update time.
 *
 * For static terrain mapping, process noise is omitted since terrain doesn't
 * change over time. Variance bounds prevent numerical issues.
 *
 * Layers created:
 * - elevation: Final height estimate (alpha * max + (1-alpha) * state)
 * - state: Kalman state estimate (x)
 * - elevation_min: Minimum observed height
 * - elevation_max: Maximum observed height
 * - variance: Kalman covariance (P)
 * - upper_bound, lower_bound: Confidence bounds (state ± 2σ)
 * - uncertainty_range: upper_bound - lower_bound
 */
class KalmanElevation {
 public:
  KalmanElevation() = default;

  /**
   * @brief Construct with parameters.
   *
   * @param min_variance Minimum variance (prevents over-confidence)
   * @param max_variance Maximum variance (caps uncertainty)
   * @param process_noise Process noise Q (maintains filter receptivity)
   * @param alpha Fixed alpha for elevation = alpha * max + (1 - alpha) * state
   */
  KalmanElevation(float min_variance, float max_variance, float process_noise,
                  float alpha = 0.5f)
      : min_variance_(min_variance),
        max_variance_(max_variance),
        process_noise_(process_noise),
        alpha_(alpha) {}

  /**
   * @brief Initialize layers on the height map.
   *
   * Creates all layers (core + derived) eagerly and caches matrix pointers.
   */
  void initialize(HeightMap& map) {
    map_ = &map;

    // Core layers
    if (!map.exists(layer::elevation)) map.add(layer::elevation, NAN);
    if (!map.exists(layer::elevation_min)) map.add(layer::elevation_min, NAN);
    if (!map.exists(layer::elevation_max)) map.add(layer::elevation_max, NAN);
    if (!map.exists(layer::state)) map.add(layer::state, NAN);
    if (!map.exists(layer::variance)) map.add(layer::variance, 0.0f);
    if (!map.exists(layer::sample_count)) map.add(layer::sample_count, 0.0f);

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

    // Min/Max update
    if (std::isnan(min_z) || z < min_z) min_z = z;
    if (std::isnan(max_z) || z > max_z) max_z = z;
  }

  /**
   * @brief Compute derived statistics after all updates.
   *
   * Computes 95% confidence bounds, denoised max, and final elevation.
   */
  void finalize() {
    // σ = sqrt(variance)
    Eigen::ArrayXXf sigma = variance_mat_->array().sqrt();

    // Confidence bounds: state ± 2σ
    *upper_mat_ = state_mat_->array() + 2.0f * sigma;
    *lower_mat_ = state_mat_->array() - 2.0f * sigma;

    // Uncertainty range = upper - lower
    *range_mat_ = upper_mat_->array() - lower_mat_->array();

    // Denoise elevation_max (copy + median filter)
    denoiseElevationMax(*map_);

    // Final elevation: fixed alpha blend
    // alpha = 0: pure estimation (state only)
    // alpha = 0.5: balanced
    // alpha = 1: max only (conservative)
    const auto& denoised_max = map_->get(layer::elevation_max_denoised);
    *elevation_mat_ =
        alpha_ * denoised_max.array() + (1.0f - alpha_) * state_mat_->array();
  }

  std::string name() const { return "KalmanElevation"; }

 private:
  float min_variance_ = 0.0001f;
  float max_variance_ = 0.01f;
  float process_noise_ = 0.0f;
  float alpha_ = 0.5f;

  // Map pointer for finalize operations
  HeightMap* map_ = nullptr;

  // Core layer matrices
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* min_mat_ = nullptr;
  grid_map::Matrix* max_mat_ = nullptr;
  grid_map::Matrix* state_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;

  // Derived layer matrices
  grid_map::Matrix* upper_mat_ = nullptr;
  grid_map::Matrix* lower_mat_ = nullptr;
  grid_map::Matrix* range_mat_ = nullptr;
};

}  // namespace height_mapping::updater

#endif  // HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_KALMAN_H
