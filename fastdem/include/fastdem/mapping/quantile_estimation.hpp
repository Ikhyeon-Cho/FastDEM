// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * quantile_estimation.hpp
 *
 * Height estimation using P² quantile algorithm.
 * Provides robust height estimation for skewed measurement distributions.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 *
 * Reference: Jain & Chlamtac (1985), "The P² Algorithm for Dynamic
 *            Calculation of Quantiles and Histograms Without Storing
 *            Observations"
 */

#ifndef FASTDEM_MAPPING_QUANTILE_ESTIMATION_HPP
#define FASTDEM_MAPPING_QUANTILE_ESTIMATION_HPP

#include <algorithm>
#include <cmath>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

/**
 * @brief Elevation layer updater using P² quantile estimation.
 *
 * Unlike mean-based estimators, P² tracks distribution quantiles online,
 * making it robust to skewed distributions common in LiDAR height measurements
 * (where viewpoint bias causes most measurements to be below true height).
 *
 * Default marker configuration {1%, 16%, 50%, 84%, 99%} provides:
 * - lower_bound at 1st percentile (q[0])
 * - upper_bound at 99th percentile (q[4])
 * - Elevation estimate at 84th percentile (q[3])
 * - σ estimate via (q[3] - q[1]) / 2
 *
 * Usage:
 * @code
 *   QuantileEstimation p2_elev(config);
 *   p2_elev.initialize(map);
 *   for (const auto& point : cloud) {
 *     p2_elev.update(index, point, variance);
 *   }
 *   p2_elev.finalize();
 * @endcode
 */
class QuantileEstimation {
 public:
  /**
   * @brief Default constructor with standard quantile markers.
   *
   * Default markers at {1%, 16%, 50%, 84%, 99%} for robust bounds estimation.
   */
  QuantileEstimation()
      : QuantileEstimation(0.01f, 0.16f, 0.50f, 0.84f, 0.99f, 3, 0.0f) {}

  /**
   * @brief Construct with P² quantile parameters.
   *
   * @param dn0 Lower bound percentile (default: 0.01 = 1%)
   * @param dn1 Lower sigma percentile (default: 0.16 = 16%)
   * @param dn2 Median percentile (default: 0.50 = 50%)
   * @param dn3 Upper sigma percentile (default: 0.84 = 84%)
   * @param dn4 Upper bound percentile (default: 0.99 = 99%)
   * @param elevation_marker Which marker to use as elevation (0-4, default: 3)
   * @param max_sample_count Maximum sample count for fading memory (0 =
   * disabled)
   */
  QuantileEstimation(float dn0, float dn1, float dn2, float dn3, float dn4,
                     int elevation_marker, float max_sample_count = 0.0f)
      : elevation_marker_(elevation_marker),
        max_sample_count_(max_sample_count) {
    dn_[0] = dn0;
    dn_[1] = dn1;
    dn_[2] = dn2;
    dn_[3] = dn3;
    dn_[4] = dn4;
  }

  /**
   * @brief Initialize P² layers on the height map.
   *
   * Creates all layers (core + P² markers + derived) eagerly and caches
   * matrix pointers.
   *
   * @param map Height map to initialize layers on
   */
  void initialize(ElevationMap& map) {
    // Common output layers
    if (!map.exists(layer::elevation)) map.add(layer::elevation, NAN);
    if (!map.exists(layer::elevation_min)) map.add(layer::elevation_min, NAN);
    if (!map.exists(layer::elevation_max)) map.add(layer::elevation_max, NAN);
    if (!map.exists(layer::variance)) map.add(layer::variance, NAN);
    if (!map.exists(layer::sample_count)) map.add(layer::sample_count, 0.0f);

    // P² marker height layers (q[0-4])
    if (!map.exists(layer::p2_q0)) map.add(layer::p2_q0, NAN);
    if (!map.exists(layer::p2_q1)) map.add(layer::p2_q1, NAN);
    if (!map.exists(layer::p2_q2)) map.add(layer::p2_q2, NAN);
    if (!map.exists(layer::p2_q3)) map.add(layer::p2_q3, NAN);
    if (!map.exists(layer::p2_q4)) map.add(layer::p2_q4, NAN);

    // P² marker position layers (n[0-4])
    if (!map.exists(layer::p2_n0)) map.add(layer::p2_n0, 0.0f);
    if (!map.exists(layer::p2_n1)) map.add(layer::p2_n1, 1.0f);
    if (!map.exists(layer::p2_n2)) map.add(layer::p2_n2, 2.0f);
    if (!map.exists(layer::p2_n3)) map.add(layer::p2_n3, 3.0f);
    if (!map.exists(layer::p2_n4)) map.add(layer::p2_n4, 4.0f);

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

    q_mat_[0] = &map.get(layer::p2_q0);
    q_mat_[1] = &map.get(layer::p2_q1);
    q_mat_[2] = &map.get(layer::p2_q2);
    q_mat_[3] = &map.get(layer::p2_q3);
    q_mat_[4] = &map.get(layer::p2_q4);

    n_mat_[0] = &map.get(layer::p2_n0);
    n_mat_[1] = &map.get(layer::p2_n1);
    n_mat_[2] = &map.get(layer::p2_n2);
    n_mat_[3] = &map.get(layer::p2_n3);
    n_mat_[4] = &map.get(layer::p2_n4);

    upper_mat_ = &map.get(layer::upper_bound);
    lower_mat_ = &map.get(layer::lower_bound);
    range_mat_ = &map.get(layer::uncertainty_range);
  }

  /**
   * @brief Update P² quantile estimate at a cell.
   *
   * @param index Pre-computed cell index
   * @param measurement Height measurement value [m]
   * @param measurement_variance Measurement variance (unused in P²)
   */
  void update(const grid_map::Index& index, float measurement,
              [[maybe_unused]] float measurement_variance) {
    const int i = index(0);
    const int j = index(1);
    const float x = measurement;

    float& count = (*count_mat_)(i, j);

    // Extract marker values for this cell
    float q[5] = {(*q_mat_[0])(i, j), (*q_mat_[1])(i, j), (*q_mat_[2])(i, j),
                  (*q_mat_[3])(i, j), (*q_mat_[4])(i, j)};
    float n[5] = {(*n_mat_[0])(i, j), (*n_mat_[1])(i, j), (*n_mat_[2])(i, j),
                  (*n_mat_[3])(i, j), (*n_mat_[4])(i, j)};

    // Run P² algorithm
    updateP2(q, n, count, x);

    // Write back marker values
    (*q_mat_[0])(i, j) = q[0];
    (*q_mat_[1])(i, j) = q[1];
    (*q_mat_[2])(i, j) = q[2];
    (*q_mat_[3])(i, j) = q[3];
    (*q_mat_[4])(i, j) = q[4];

    (*n_mat_[0])(i, j) = n[0];
    (*n_mat_[1])(i, j) = n[1];
    (*n_mat_[2])(i, j) = n[2];
    (*n_mat_[3])(i, j) = n[3];
    (*n_mat_[4])(i, j) = n[4];

    // Track true min/max (separate from P² quantiles)
    float& min_z = (*min_mat_)(i, j);
    float& max_z = (*max_mat_)(i, j);
    if (std::isnan(min_z) || x < min_z) min_z = x;
    if (std::isnan(max_z) || x > max_z) max_z = x;

    // Elevation: q[marker] when P² is active, latest measurement otherwise
    const int elev_idx = std::clamp(elevation_marker_, 0, 4);
    (*elevation_mat_)(i, j) = (count >= 5.0f) ? q[elev_idx] : x;
  }

  /**
   * @brief Compute derived statistics after all updates.
   *
   * Maps P² markers to standard output layers:
   * - elevation: q[elevation_marker] (default: 84th percentile)
   * - elevation_min/max: True min/max (tracked in update())
   * - lower_bound: q[0] (1st percentile)
   * - upper_bound: q[4] (99th percentile)
   * - variance: ((q[3] - q[1]) / 2)² (σ² estimate)
   */
  void finalize() {
    const int elev_idx = std::clamp(elevation_marker_, 0, 4);

    // Vectorized computation using Eigen arrays
    auto q1 = q_mat_[1]->array();
    auto q3 = q_mat_[3]->array();

    // Map markers to output layers
    // Note: min_mat_, max_mat_ are already set in update() (true min/max)
    *elevation_mat_ = *q_mat_[elev_idx];  // elevation = q[elev_idx]

    // σ estimate: (84th - 16th) / 2
    Eigen::ArrayXXf sigma = (q3 - q1) / 2.0f;
    variance_mat_->array() = sigma.square();

    // Confidence bounds: directly from quantiles (no normal distribution
    // assumption)
    *lower_mat_ = *q_mat_[0];  // 1st percentile
    *upper_mat_ = *q_mat_[4];  // 99th percentile
    range_mat_->array() = upper_mat_->array() - lower_mat_->array();
  }

 private:
  /**
   * @brief P² algorithm core update.
   *
   * @param q Marker heights (5 elements)
   * @param n Marker positions (5 elements)
   * @param count Sample count (updated in-place)
   * @param x New measurement
   */
  void updateP2(float* q, float* n, float& count, float x) {
    // Handle uninitialized cell (NaN count after grid_map::move())
    if (std::isnan(count) || count < 0.0f) {
      count = 0.0f;
    }

    // Phase 1: Initialization (first 5 observations)
    if (count < 5.0f) {
      int idx = static_cast<int>(count);
      q[idx] = x;
      count += 1.0f;

      // Sort and initialize positions after 5th observation
      if (count >= 5.0f) {
        std::sort(q, q + 5);
        for (int i = 0; i < 5; ++i) {
          n[i] = static_cast<float>(i);
        }
      }
      return;
    }

    // Phase 2: P² update
    // Step 1: Find cell k where x falls, update extreme markers if needed
    int k;
    if (x < q[0]) {
      q[0] = x;
      k = 0;
    } else if (x < q[1]) {
      k = 0;
    } else if (x < q[2]) {
      k = 1;
    } else if (x < q[3]) {
      k = 2;
    } else if (x <= q[4]) {
      k = 3;
    } else {
      q[4] = x;
      k = 3;
    }

    // Step 2: Increment positions of markers k+1 to 4
    for (int i = k + 1; i < 5; ++i) {
      n[i] += 1.0f;
    }

    // Step 3: Update desired positions
    float n_prime[5];
    for (int i = 0; i < 5; ++i) {
      n_prime[i] = dn_[i] * count;
    }

    count += 1.0f;

    // Fading Memory: rescale count and marker positions when exceeding limit
    // This maintains responsiveness to new data in dynamic environments
    if (max_sample_count_ > 0.0f && count > max_sample_count_) {
      const float scale = max_sample_count_ / count;
      for (int i = 0; i < 5; ++i) {
        n[i] *= scale;
      }
      count = max_sample_count_;
    }

    // Step 4: Adjust marker heights for markers 1, 2, 3
    for (int i = 1; i < 4; ++i) {
      float d = n_prime[i] - n[i];

      if ((d >= 1.0f && n[i + 1] - n[i] > 1.0f) ||
          (d <= -1.0f && n[i - 1] - n[i] < -1.0f)) {
        int sign = (d >= 0.0f) ? 1 : -1;

        // Try parabolic interpolation
        float q_new = parabolic(q, n, i, sign);

        // Check if parabolic result is valid (monotonicity)
        if (q[i - 1] < q_new && q_new < q[i + 1]) {
          q[i] = q_new;
        } else {
          // Fallback to linear interpolation
          q[i] = linear(q, n, i, sign);
        }

        n[i] += static_cast<float>(sign);
      }
    }
  }

  /**
   * @brief Parabolic (P²) interpolation for marker adjustment.
   */
  float parabolic(const float* q, const float* n, int i, int sign) const {
    float qi = q[i];
    float qim1 = q[i - 1];
    float qip1 = q[i + 1];
    float ni = n[i];
    float nim1 = n[i - 1];
    float nip1 = n[i + 1];
    float s = static_cast<float>(sign);

    float term1 = (ni - nim1 + s) * (qip1 - qi) / (nip1 - ni);
    float term2 = (nip1 - ni - s) * (qi - qim1) / (ni - nim1);

    return qi + s * (term1 + term2) / (nip1 - nim1);
  }

  /**
   * @brief Linear interpolation fallback for marker adjustment.
   */
  float linear(const float* q, const float* n, int i, int sign) const {
    int j = i + sign;
    return q[i] + static_cast<float>(sign) * (q[j] - q[i]) / (n[j] - n[i]);
  }

  int elevation_marker_ = 3;    // Which marker to use as elevation output (0-4)
  float dn_[5];                 // Desired position increments
  float max_sample_count_ = 0;  // Max count for fading memory (0 = disabled)

  // Common output layer matrices
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* min_mat_ = nullptr;
  grid_map::Matrix* max_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;

  // P² marker matrices
  grid_map::Matrix* q_mat_[5] = {nullptr};
  grid_map::Matrix* n_mat_[5] = {nullptr};

  // Derived layer matrices
  grid_map::Matrix* upper_mat_ = nullptr;
  grid_map::Matrix* lower_mat_ = nullptr;
  grid_map::Matrix* range_mat_ = nullptr;
};

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_QUANTILE_ESTIMATION_HPP
