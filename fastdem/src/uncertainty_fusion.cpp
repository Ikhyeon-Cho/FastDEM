// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * uncertainty_fusion.cpp
 *
 * Implementation of uncertainty fusion using bilateral filter + weighted ECDF.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/postprocess/uncertainty_fusion.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

namespace {

/**
 * @brief Simple weighted ECDF for quantile extraction.
 *
 * Accumulates (value, weight) pairs and computes weighted quantiles.
 * Uses sorting-based approach (O(n log n)) which is acceptable for
 * small neighbor sets (~50 cells max for typical search radii).
 */
class SimpleWeightedECDF {
 public:
  void reserve(size_t n) { samples_.reserve(n); }

  void add(float value, float weight) {
    if (weight > 1e-6f && std::isfinite(value)) {
      samples_.push_back({value, weight});
    }
  }

  void clear() { samples_.clear(); }

  size_t size() const { return samples_.size(); }

  /**
   * @brief Compute weighted quantile.
   *
   * Algorithm:
   * 1. Sort samples by value
   * 2. Compute cumulative weight
   * 3. Find value where cumulative_weight/total_weight >= p
   *
   * @param p Quantile (0.0 to 1.0)
   * @return Weighted quantile value, or NAN if no samples
   */
  float quantile(float p) {
    if (samples_.empty()) return NAN;
    if (samples_.size() == 1) return samples_[0].value;

    // Sort by value (in-place; caller should not reuse after this)
    std::sort(samples_.begin(), samples_.end(),
              [](const Sample& a, const Sample& b) {
                return a.value < b.value;
              });

    // Compute total weight
    float total_weight = 0.0f;
    for (const auto& s : samples_) {
      total_weight += s.weight;
    }

    if (total_weight <= 0.0f) return NAN;

    // Find quantile via cumulative weight
    const float target = p * total_weight;
    float cumulative = 0.0f;

    for (const auto& s : samples_) {
      cumulative += s.weight;
      if (cumulative >= target) {
        return s.value;
      }
    }

    return samples_.back().value;
  }

 private:
  struct Sample {
    float value;
    float weight;
  };
  std::vector<Sample> samples_;
};

}  // namespace

void applyUncertaintyFusion(ElevationMap& map,
                            const config::UncertaintyFusion& config) {
  if (!config.enabled) return;

  // Validate required layers (bounds from estimator finalize)
  if (!map.exists(layer::upper_bound) || !map.exists(layer::lower_bound)) {
    spdlog::warn(
        "[UncertaintyFusion] Missing required layers (upper_bound, "
        "lower_bound).");
    return;
  }

  // Get layer references
  auto& upper_mat = map.get(layer::upper_bound);
  auto& lower_mat = map.get(layer::lower_bound);

  const auto reg = map.region(config.search_radius);

  // Pre-compute Gaussian constant for spatial weight
  const float inv_2sigma_spatial_sq =
      1.0f / (2.0f * config.spatial_sigma * config.spatial_sigma);

  // Buffers for double-buffering (copy existing values as fallback)
  Eigen::MatrixXf upper_buffer = upper_mat;
  Eigen::MatrixXf lower_buffer = lower_mat;

  // Two separate ECDFs for lower and upper bounds
  SimpleWeightedECDF lower_ecdf;
  SimpleWeightedECDF upper_ecdf;
  lower_ecdf.reserve(reg.entries.size());
  upper_ecdf.reserve(reg.entries.size());

  for (auto cell : map.cells()) {
    const float center_upper = upper_mat(cell.index);
    const float center_lower = lower_mat(cell.index);

    // Skip invalid cells
    if (!std::isfinite(center_upper) || !std::isfinite(center_lower)) continue;

    lower_ecdf.clear();
    upper_ecdf.clear();
    int valid_count = 0;

    // Gather weighted samples from neighbors
    for (auto n : map.neighbors(cell, reg)) {
      const float neighbor_upper = upper_mat(n.index);
      const float neighbor_lower = lower_mat(n.index);

      // Skip invalid neighbors
      if (!std::isfinite(neighbor_upper) || !std::isfinite(neighbor_lower))
        continue;

      // Spatial weight (Gaussian distance decay)
      const float w_spatial = std::exp(-n.dist_sq * inv_2sigma_spatial_sq);

      // Inverse range weight (narrow bounds = more certain → higher weight)
      constexpr float epsilon = 1e-4f;
      const float range = neighbor_upper - neighbor_lower;
      const float w_range = 1.0f / (range + epsilon);

      const float weight = w_spatial * w_range;

      // Add estimator-computed bounds directly to ECDF
      lower_ecdf.add(neighbor_lower, weight);
      upper_ecdf.add(neighbor_upper, weight);

      ++valid_count;
    }

    // Compute fused bounds if enough neighbors
    if (valid_count >= config.min_valid_neighbors) {
      const float lower = lower_ecdf.quantile(config.quantile_lower);
      const float upper = upper_ecdf.quantile(config.quantile_upper);
      if (std::isfinite(lower) && std::isfinite(upper)) {
        upper_buffer(cell.index) = upper;
        lower_buffer(cell.index) = lower;
      }
    }
  }

  // Copy buffers to output
  upper_mat = upper_buffer;
  lower_mat = lower_buffer;
}

}  // namespace fastdem
