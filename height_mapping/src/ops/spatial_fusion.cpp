/*
 * spatial_fusion.cpp
 *
 * Implementation of spatial uncertainty fusion using bilateral filter + weighted
 * ECDF.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/ops/spatial_fusion.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "height_mapping/core/layers.h"

namespace height_mapping::ops {

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

  void clear() {
    samples_.clear();
  }

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
  float quantile(float p) const {
    if (samples_.empty()) return NAN;
    if (samples_.size() == 1) return samples_[0].value;

    // Sort by value (makes a copy for non-destructive operation)
    auto sorted = samples_;
    std::sort(sorted.begin(), sorted.end(),
              [](const Sample& a, const Sample& b) { return a.value < b.value; });

    // Compute total weight
    float total_weight = 0.0f;
    for (const auto& s : sorted) {
      total_weight += s.weight;
    }

    if (total_weight <= 0.0f) return NAN;

    // Find quantile via cumulative weight
    const float target = p * total_weight;
    float cumulative = 0.0f;

    for (const auto& s : sorted) {
      cumulative += s.weight;
      if (cumulative >= target) {
        return s.value;
      }
    }

    return sorted.back().value;
  }

 private:
  struct Sample {
    float value;
    float weight;
  };
  std::vector<Sample> samples_;
};

/**
 * @brief Pre-computed neighbor offset with squared distance.
 */
struct NeighborOffset {
  int dr;
  int dc;
  float dist_sq;  ///< Pre-computed squared distance in meters
};

/**
 * @brief Pre-compute neighbor offsets within circular search radius.
 *
 * @param search_radius Search radius in meters
 * @param resolution Grid resolution in meters
 * @param offsets Output vector of neighbor offsets
 */
void precomputeNeighborOffsets(float search_radius, float resolution,
                               std::vector<NeighborOffset>& offsets) {
  offsets.clear();
  const int radius_cells = static_cast<int>(std::ceil(search_radius / resolution));
  const float radius_sq = search_radius * search_radius;

  for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
    for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
      const float dist_m =
          resolution * std::sqrt(static_cast<float>(dr * dr + dc * dc));
      const float dist_sq = dist_m * dist_m;

      if (dist_sq <= radius_sq) {
        offsets.push_back({dr, dc, dist_sq});
      }
    }
  }
}

}  // namespace

void applySpatialFusion(HeightMap& map, const config::SpatialFusion& config) {
  if (!config.enabled) return;

  // Validate required layers
  if (!map.exists(layer::state) || !map.exists(layer::variance)) {
    return;  // Missing required layers
  }

  // Ensure output layers exist
  if (!map.exists(layer::upper_bound)) map.add(layer::upper_bound, NAN);
  if (!map.exists(layer::lower_bound)) map.add(layer::lower_bound, NAN);
  if (!map.exists(layer::uncertainty_range)) {
    map.add(layer::uncertainty_range, NAN);
  }

  // Get layer references
  const auto& state_mat = map.get(layer::state);
  const auto& variance_mat = map.get(layer::variance);
  auto& upper_mat = map.get(layer::upper_bound);
  auto& lower_mat = map.get(layer::lower_bound);
  auto& range_mat = map.get(layer::uncertainty_range);

  const int rows = static_cast<int>(state_mat.rows());
  const int cols = static_cast<int>(state_mat.cols());
  const float resolution = map.getResolution();

  // Pre-compute neighbor offsets within search_radius
  std::vector<NeighborOffset> neighbor_offsets;
  precomputeNeighborOffsets(config.search_radius, resolution, neighbor_offsets);

  // Pre-compute Gaussian constant for spatial weight
  const float inv_2sigma_spatial_sq =
      1.0f / (2.0f * config.spatial_sigma * config.spatial_sigma);

  // Buffers for double-buffering (copy existing values as fallback)
  Eigen::MatrixXf upper_buffer = upper_mat;
  Eigen::MatrixXf lower_buffer = lower_mat;
  Eigen::MatrixXf range_buffer = range_mat;

  // Two separate ECDFs for lower and upper bounds (elevation_mapping style)
  SimpleWeightedECDF lower_ecdf;
  SimpleWeightedECDF upper_ecdf;
  lower_ecdf.reserve(neighbor_offsets.size());
  upper_ecdf.reserve(neighbor_offsets.size());

  // Process each cell
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const float center_h = state_mat(r, c);
      const float center_var = variance_mat(r, c);

      // Skip invalid cells
      if (!std::isfinite(center_h) || center_var <= 0.0f) {
        continue;
      }

      lower_ecdf.clear();
      upper_ecdf.clear();
      int valid_count = 0;

      // Gather weighted samples from neighbors
      for (const auto& offset : neighbor_offsets) {
        const int nr = r + offset.dr;
        const int nc = c + offset.dc;

        // Boundary check
        if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) {
          continue;
        }

        const float neighbor_h = state_mat(nr, nc);
        const float neighbor_var = variance_mat(nr, nc);

        // Skip invalid neighbors
        if (!std::isfinite(neighbor_h) || neighbor_var <= 0.0f) {
          continue;
        }

        // Weight computation
        // Spatial weight (Gaussian distance decay)
        const float w_spatial = std::exp(-offset.dist_sq * inv_2sigma_spatial_sq);

        // Inverse variance weight (confident cells dominate)
        constexpr float epsilon = 1e-4f;  // Prevents division by zero
        const float w_variance = 1.0f / (neighbor_var + epsilon);

        const float weight = w_spatial * w_variance;

        // Add to separate distributions (mean +/- 2*sigma)
        const float sigma = std::sqrt(neighbor_var);
        lower_ecdf.add(neighbor_h - 2.0f * sigma, weight);
        upper_ecdf.add(neighbor_h + 2.0f * sigma, weight);

        valid_count++;
      }

      // Compute fused bounds if enough neighbors
      // Otherwise, keep existing values from finalize()
      if (valid_count >= config.min_valid_neighbors) {
        const float lower = lower_ecdf.quantile(config.quantile_lower);
        const float upper = upper_ecdf.quantile(config.quantile_upper);
        if (std::isfinite(lower) && std::isfinite(upper)) {
          upper_buffer(r, c) = upper;
          lower_buffer(r, c) = lower;
          range_buffer(r, c) = upper - lower;
        }
        // else: keep existing values (already copied)
      }
      // else: keep existing values (already copied)
    }
  }

  // Copy buffers to output
  upper_mat = upper_buffer;
  lower_mat = lower_buffer;
  range_mat = range_buffer;
}

}  // namespace height_mapping::ops
