// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * mean_estimation.hpp
 *
 * Height estimation using Welford's online mean algorithm.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_MAPPING_MEAN_ESTIMATION_HPP
#define FASTDEM_MAPPING_MEAN_ESTIMATION_HPP

#include <cmath>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

/**
 * @brief Elevation updater using Welford's online mean algorithm.
 *
 * Computes running mean and sample variance using numerically stable method.
 * Elevation output is the running mean directly.
 *
 * Layers created:
 * - elevation: Running mean of measurements
 * - elevation_min/max: Observed height range
 * - variance: Sample variance (Welford)
 * - sample_count: Number of measurements
 * - standard_error: SE = sqrt(variance / count)
 * - conf_interval_95: 1.96 * SE
 * - upper_bound, lower_bound: elevation ± 1.96 * SE
 * - uncertainty_range: upper_bound - lower_bound
 *
 * Reference: Welford (1962), "Note on a Method for Calculating Corrected
 *            Sums of Squares and Products"
 */
class MeanEstimation {
 public:
  MeanEstimation() = default;

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

    // Derived layers (eagerly created for finalize)
    if (!map.exists(layer::standard_error)) map.add(layer::standard_error, NAN);
    if (!map.exists(layer::conf_interval_95))
      map.add(layer::conf_interval_95, NAN);
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
    se_mat_ = &map.get(layer::standard_error);
    ci_mat_ = &map.get(layer::conf_interval_95);
    upper_mat_ = &map.get(layer::upper_bound);
    lower_mat_ = &map.get(layer::lower_bound);
    range_mat_ = &map.get(layer::uncertainty_range);
  }

  /**
   * @brief Update elevation estimate at a cell using Welford's algorithm.
   *
   * @param index Pre-computed cell index
   * @param measurement Height measurement value [m]
   * @param measurement_variance Measurement variance (unused in mean
   * estimation)
   */
  void update(const grid_map::Index& index, float measurement,
              [[maybe_unused]] float measurement_variance) {
    const int i = index(0);
    const int j = index(1);

    // Map layer references (elevation = running mean)
    float& elevation = (*elevation_mat_)(i, j);
    float& variance = (*variance_mat_)(i, j);
    float& min_z = (*min_mat_)(i, j);
    float& max_z = (*max_mat_)(i, j);
    float& count = (*count_mat_)(i, j);

    const float z = measurement;

    // Welford's online algorithm
    if (std::isnan(elevation)) {
      // First measurement
      elevation = z;
      variance = 0.0f;
      count = 1.0f;
    } else {
      count += 1.0f;

      const float delta = z - elevation;
      const float new_mean = elevation + (delta / count);
      const float delta2 = z - new_mean;

      // Update sample variance using numerically stable computation
      if (count <= 2.0f) {
        variance = delta * delta2;
      } else {
        float m2 = variance * (count - 2.0f);
        m2 += delta * delta2;
        variance = m2 / (count - 1.0f);
      }

      elevation = new_mean;
    }

    // Min/Max update
    if (std::isnan(min_z) || z < min_z) min_z = z;
    if (std::isnan(max_z) || z > max_z) max_z = z;
  }

  /**
   * @brief Compute derived statistics after all updates.
   *
   * Computes standard_error, conf_interval_95, and confidence bounds.
   */
  void finalize() {
    // Vectorized computation: SE = sqrt(variance / count)
    *se_mat_ = (variance_mat_->array() / count_mat_->array()).sqrt();

    // CI_95 = 1.96 * SE
    *ci_mat_ = 1.96f * se_mat_->array();

    // Confidence bounds: elevation ± 1.96 * SE
    *upper_mat_ = elevation_mat_->array() + ci_mat_->array();
    *lower_mat_ = elevation_mat_->array() - ci_mat_->array();

    // Uncertainty range = upper - lower
    *range_mat_ = upper_mat_->array() - lower_mat_->array();
  }

  std::string name() const { return "MeanEstimation"; }

 private:
  // Common output layer matrices
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* min_mat_ = nullptr;
  grid_map::Matrix* max_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;

  // Derived layer matrices
  grid_map::Matrix* se_mat_ = nullptr;
  grid_map::Matrix* ci_mat_ = nullptr;
  grid_map::Matrix* upper_mat_ = nullptr;
  grid_map::Matrix* lower_mat_ = nullptr;
  grid_map::Matrix* range_mat_ = nullptr;
};

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_MEAN_ESTIMATION_HPP
