/*
 * spatial_fusion.h
 *
 * Configuration for spatial uncertainty fusion using inverse variance weighted
 * ECDF.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_SPATIAL_FUSION_H
#define HEIGHT_MAPPING_CONFIG_SPATIAL_FUSION_H

namespace height_mapping::config {

/**
 * @brief Spatial uncertainty fusion using inverse variance weighted ECDF.
 *
 * Computes neighborhood-aware uncertainty bounds by fusing nearby cells with
 * inverse variance weighting (confident cells dominate).
 *
 * Algorithm:
 * 1. For each cell, gather neighbors within search_radius
 * 2. Compute weights: W = exp(-d²/(2σ_spatial²)) × (1/variance)
 * 3. Build weighted ECDF from lower/upper samples: μ_i ± 2σ_i with weight W_i
 * 4. Extract quantile_lower and quantile_upper from weighted ECDF
 * 5. uncertainty_range = upper - lower
 */
struct SpatialFusion {
  /// Enable spatial uncertainty fusion (default: disabled)
  bool enabled = false;

  /// Neighbor search radius [m] (typically 1-2 cells: 0.1-0.2m)
  float search_radius = 0.15f;

  /// Spatial Gaussian sigma [m] - controls distance weight decay
  float spatial_sigma = 0.05f;

  /// Lower quantile for uncertainty bound (1% = robust to outliers)
  float quantile_lower = 0.01f;

  /// Upper quantile for uncertainty bound (99% = robust to outliers)
  float quantile_upper = 0.99f;

  /// Minimum valid neighbors required (skip cells with fewer)
  int min_valid_neighbors = 3;
};

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_SPATIAL_FUSION_H
