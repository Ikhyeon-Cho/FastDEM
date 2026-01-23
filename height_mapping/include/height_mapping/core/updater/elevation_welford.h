/*
 * elevation_welford.h
 *
 * Elevation layer updater using Welford's online mean algorithm.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_WELFORD_H
#define HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_WELFORD_H

#include <cmath>

#include "height_mapping/core/layers.h"
#include "height_mapping/core/map.h"
#include "height_mapping/core/updater/utility.h"

namespace height_mapping::updater {

/**
 * @brief Elevation updater using Welford's online mean algorithm.
 *
 * Computes running mean and sample variance using numerically stable method.
 * Final elevation is computed as weighted combination of mean and robust max.
 *
 * Layers created:
 * - elevation: Final height estimate (alpha * max + (1-alpha) * mean)
 * - mean: Running mean of measurements
 * - elevation_min: Minimum observed height
 * - elevation_max: Maximum observed height
 * - variance: Sample variance (Welford)
 * - sample_count: Number of measurements
 * - standard_error: SE = sqrt(variance / count)
 * - conf_interval_95: 1.96 * SE
 *
 * Reference: Welford (1962), "Note on a Method for Calculating Corrected
 *            Sums of Squares and Products"
 */
class WelfordElevation {
 public:
  WelfordElevation() = default;

  /**
   * @brief Construct with parameters.
   *
   * @param alpha Fixed alpha for elevation = alpha * max + (1 - alpha) * mean
   */
  explicit WelfordElevation(float alpha) : alpha_(alpha) {}

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
    if (!map.exists(layer::mean)) map.add(layer::mean, NAN);
    if (!map.exists(layer::variance)) map.add(layer::variance, 0.0f);
    if (!map.exists(layer::sample_count)) map.add(layer::sample_count, 0.0f);

    // Derived layers (eagerly created for finalize)
    if (!map.exists(layer::standard_error)) map.add(layer::standard_error, NAN);
    if (!map.exists(layer::conf_interval_95))
      map.add(layer::conf_interval_95, NAN);

    // Cache all matrix pointers
    elevation_mat_ = &map.get(layer::elevation);
    min_mat_ = &map.get(layer::elevation_min);
    max_mat_ = &map.get(layer::elevation_max);
    mean_mat_ = &map.get(layer::mean);
    variance_mat_ = &map.get(layer::variance);
    count_mat_ = &map.get(layer::sample_count);
    se_mat_ = &map.get(layer::standard_error);
    ci_mat_ = &map.get(layer::conf_interval_95);
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

    // Map layer references
    float& mean = (*mean_mat_)(i, j);
    float& variance = (*variance_mat_)(i, j);
    float& min_z = (*min_mat_)(i, j);
    float& max_z = (*max_mat_)(i, j);
    float& count = (*count_mat_)(i, j);

    const float z = measurement;

    // Welford's online algorithm
    if (std::isnan(mean)) {
      // First measurement
      mean = z;
      variance = 0.0f;
      count = 1.0f;
    } else {
      count += 1.0f;

      const float delta = z - mean;
      const float new_mean = mean + (delta / count);
      const float delta2 = z - new_mean;

      // Update sample variance using numerically stable computation
      if (count <= 2.0f) {
        variance = delta * delta2;
      } else {
        float m2 = variance * (count - 2.0f);
        m2 += delta * delta2;
        variance = m2 / (count - 1.0f);
      }

      mean = new_mean;
    }

    // Min/Max update
    if (std::isnan(min_z) || z < min_z) min_z = z;
    if (std::isnan(max_z) || z > max_z) max_z = z;
  }

  /**
   * @brief Compute derived statistics after all updates.
   *
   * Computes standard_error, conf_interval_95, denoised max, and final
   * elevation.
   */
  void finalize() {
    // Vectorized computation: SE = sqrt(variance / count)
    *se_mat_ = (variance_mat_->array() / count_mat_->array()).sqrt();

    // CI_95 = 1.96 * SE
    *ci_mat_ = 1.96f * se_mat_->array();

    // Denoise elevation_max (copy + median filter)
    denoiseElevationMax(*map_);

    // Final elevation: fixed alpha blend
    // alpha = 0: pure estimation (mean only)
    // alpha = 0.5: balanced
    // alpha = 1: max only (conservative)
    const auto& denoised_max = map_->get(layer::elevation_max_denoised);
    *elevation_mat_ =
        alpha_ * denoised_max.array() + (1.0f - alpha_) * mean_mat_->array();
  }

  std::string name() const { return "WelfordElevation"; }

 private:
  float alpha_ = 0.5f;

  // Map pointer for finalize operations
  HeightMap* map_ = nullptr;

  // Core layer matrices
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* min_mat_ = nullptr;
  grid_map::Matrix* max_mat_ = nullptr;
  grid_map::Matrix* mean_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;

  // Derived layer matrices
  grid_map::Matrix* se_mat_ = nullptr;
  grid_map::Matrix* ci_mat_ = nullptr;
};

}  // namespace height_mapping::updater

#endif  // HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_WELFORD_H
