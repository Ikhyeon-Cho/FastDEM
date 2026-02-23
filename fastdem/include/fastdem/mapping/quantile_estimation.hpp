// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * quantile_estimation.hpp
 *
 * Height estimation using P² quantile algorithm.
 * Provides robust height estimation for skewed measurement distributions.
 * Reference: Jain & Chlamtac (1985), "The P² Algorithm for Dynamic
 *            Calculation of Quantiles and Histograms Without Storing
 *            Observations"
 */

#ifndef FASTDEM_MAPPING_QUANTILE_ESTIMATION_HPP
#define FASTDEM_MAPPING_QUANTILE_ESTIMATION_HPP

#include <algorithm>
#include <cassert>
#include <cmath>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

namespace layer {
constexpr auto p2_q0 = "_p2_q0";
constexpr auto p2_q1 = "_p2_q1";
constexpr auto p2_q2 = "_p2_q2";
constexpr auto p2_q3 = "_p2_q3";
constexpr auto p2_q4 = "_p2_q4";
constexpr auto p2_n0 = "_p2_n0";
constexpr auto p2_n1 = "_p2_n1";
constexpr auto p2_n2 = "_p2_n2";
constexpr auto p2_n3 = "_p2_n3";
constexpr auto p2_n4 = "_p2_n4";
}  // namespace layer

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
 *   P2Quantile p2_elev(config);
 *   p2_elev.ensureLayers(map);  // once
 *   p2_elev.bind(map);          // each frame
 *   for (const auto& point : cloud) {
 *     p2_elev.update(index, point, variance);
 *   }
 *   p2_elev.computeBounds();
 * @endcode
 */
class P2Quantile {
 public:
  /**
   * @brief Default constructor with standard quantile markers.
   *
   * Default markers at {1%, 16%, 50%, 84%, 99%} for robust bounds estimation.
   */
  P2Quantile() : P2Quantile(0.01f, 0.16f, 0.50f, 0.84f, 0.99f, 3, 0.0f) {}

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
  P2Quantile(float dn0, float dn1, float dn2, float dn3, float dn4,
             int elevation_marker, float max_sample_count = 0.0f)
      : elevation_marker_(std::clamp(elevation_marker, 0, 4)),
        max_sample_count_(std::max(max_sample_count, 0.0f)) {
    dn_[0] = std::clamp(dn0, 0.0f, 1.0f);
    dn_[1] = std::clamp(dn1, 0.0f, 1.0f);
    dn_[2] = std::clamp(dn2, 0.0f, 1.0f);
    dn_[3] = std::clamp(dn3, 0.0f, 1.0f);
    dn_[4] = std::clamp(dn4, 0.0f, 1.0f);
    // Enforce monotonic ordering: dn[i] >= dn[i-1]
    for (int i = 1; i < 5; ++i) dn_[i] = std::max(dn_[i], dn_[i - 1]);
  }

  /// Create required layers on the map. Call once before first update.
  void ensureLayers(ElevationMap& map) {
    if (!map.exists(layer::variance)) map.add(layer::variance, NAN);
    if (!map.exists(layer::n_points)) map.add(layer::n_points, 0.0f);

    if (!map.exists(layer::p2_q0)) map.add(layer::p2_q0, NAN);
    if (!map.exists(layer::p2_q1)) map.add(layer::p2_q1, NAN);
    if (!map.exists(layer::p2_q2)) map.add(layer::p2_q2, NAN);
    if (!map.exists(layer::p2_q3)) map.add(layer::p2_q3, NAN);
    if (!map.exists(layer::p2_q4)) map.add(layer::p2_q4, NAN);

    if (!map.exists(layer::p2_n0)) map.add(layer::p2_n0, 0.0f);
    if (!map.exists(layer::p2_n1)) map.add(layer::p2_n1, 1.0f);
    if (!map.exists(layer::p2_n2)) map.add(layer::p2_n2, 2.0f);
    if (!map.exists(layer::p2_n3)) map.add(layer::p2_n3, 3.0f);
    if (!map.exists(layer::p2_n4)) map.add(layer::p2_n4, 4.0f);

    if (!map.exists(layer::upper_bound)) map.add(layer::upper_bound, NAN);
    if (!map.exists(layer::lower_bound)) map.add(layer::lower_bound, NAN);
  }

  /// Cache matrix pointers. Call each frame before update loop.
  void bind(ElevationMap& map) {
    elevation_mat_ = &map.get(layer::elevation);
    variance_mat_ = &map.get(layer::variance);
    count_mat_ = &map.get(layer::n_points);

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
    bound_ = true;
  }

  /// Update P² quantile estimate at a single cell.
  void update(const grid_map::Index& index, float measurement,
              [[maybe_unused]] float measurement_variance) {
    assert(bound_ && "P2Quantile::bind() must be called before update()");
    const int i = index(0);
    const int j = index(1);
    const float x = measurement;
    float& count = (*count_mat_)(i, j);

    float q[5] = {(*q_mat_[0])(i, j), (*q_mat_[1])(i, j), (*q_mat_[2])(i, j),
                  (*q_mat_[3])(i, j), (*q_mat_[4])(i, j)};
    float n[5] = {(*n_mat_[0])(i, j), (*n_mat_[1])(i, j), (*n_mat_[2])(i, j),
                  (*n_mat_[3])(i, j), (*n_mat_[4])(i, j)};

    updateP2(q, n, count, x);

    for (int k = 0; k < 5; ++k) {
      (*q_mat_[k])(i, j) = q[k];
      (*n_mat_[k])(i, j) = n[k];
    }

    const int elev_idx = std::clamp(elevation_marker_, 0, 4);
    (*elevation_mat_)(i, j) = (count >= 5.0f) ? q[elev_idx] : x;
  }

  /// Compute bounds and variance at a single cell.
  void computeBounds(const grid_map::Index& index) {
    assert(bound_ &&
           "P2Quantile::bind() must be called before computeBounds()");
    const int i = index(0);
    const int j = index(1);
    const int elev_idx = std::clamp(elevation_marker_, 0, 4);
    (*elevation_mat_)(i, j) = (*q_mat_[elev_idx])(i, j);
    const float sigma =
        ((*q_mat_[3])(i, j) - (*q_mat_[1])(i, j)) / 2.0f;
    (*variance_mat_)(i, j) = sigma * sigma;
    (*lower_mat_)(i, j) = (*q_mat_[0])(i, j);
    (*upper_mat_)(i, j) = (*q_mat_[4])(i, j);
  }

 private:
  /// P² algorithm core. See Jain & Chlamtac (1985).
  void updateP2(float* q, float* n, float& count, float x) {
    if (std::isnan(count) || count < 0.0f) count = 0.0f;

    // Phase 1: collect first 5 observations
    if (count < 5.0f) {
      q[static_cast<int>(count)] = x;
      count += 1.0f;
      if (count >= 5.0f) {
        std::sort(q, q + 5);
        for (int i = 0; i < 5; ++i) n[i] = static_cast<float>(i);
      }
      return;
    }

    // Phase 2: find interval k, update extreme markers
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

    for (int i = k + 1; i < 5; ++i) n[i] += 1.0f;

    float n_prime[5];
    for (int i = 0; i < 5; ++i) n_prime[i] = dn_[i] * count;

    count += 1.0f;

    // Fading memory: rescale to maintain responsiveness in dynamic environments
    if (max_sample_count_ > 0.0f && count > max_sample_count_) {
      const float scale = max_sample_count_ / count;
      for (int i = 0; i < 5; ++i) n[i] *= scale;
      count = max_sample_count_;
    }

    // Adjust interior markers (1, 2, 3)
    for (int i = 1; i < 4; ++i) {
      float d = n_prime[i] - n[i];
      if ((d >= 1.0f && n[i + 1] - n[i] > 1.0f) ||
          (d <= -1.0f && n[i - 1] - n[i] < -1.0f)) {
        int sign = (d >= 0.0f) ? 1 : -1;
        float q_new = parabolic(q, n, i, sign);
        q[i] = (q[i - 1] < q_new && q_new < q[i + 1]) ? q_new
                                                      : linear(q, n, i, sign);
        n[i] += static_cast<float>(sign);
      }
    }
  }

  float parabolic(const float* q, const float* n, int i, int sign) const {
    const float d_right = n[i + 1] - n[i];
    const float d_left = n[i] - n[i - 1];
    const float d_span = n[i + 1] - n[i - 1];
    if (d_right == 0.0f || d_left == 0.0f || d_span == 0.0f) return q[i];
    float s = static_cast<float>(sign);
    float t1 = (d_left + s) * (q[i + 1] - q[i]) / d_right;
    float t2 = (d_right - s) * (q[i] - q[i - 1]) / d_left;
    return q[i] + s * (t1 + t2) / d_span;
  }

  float linear(const float* q, const float* n, int i, int sign) const {
    int j = i + sign;
    const float dn = n[j] - n[i];
    if (dn == 0.0f) return q[i];
    return q[i] + static_cast<float>(sign) * (q[j] - q[i]) / dn;
  }

  int elevation_marker_ = 3;    // Which marker to use as elevation output (0-4)
  float dn_[5];                 // Desired position increments
  float max_sample_count_ = 0;  // Max count for fading memory (0 = disabled)

  // Common output layer matrices
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;

  // P² marker matrices
  grid_map::Matrix* q_mat_[5] = {nullptr};
  grid_map::Matrix* n_mat_[5] = {nullptr};

  // Derived layer matrices
  grid_map::Matrix* upper_mat_ = nullptr;
  grid_map::Matrix* lower_mat_ = nullptr;

  bool bound_ = false;
};

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_QUANTILE_ESTIMATION_HPP
