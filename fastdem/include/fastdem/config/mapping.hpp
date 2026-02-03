// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * mapping.hpp
 *
 * Mapping configuration: mode, height estimation algorithm, and parameters.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_CONFIG_MAPPING_HPP
#define FASTDEM_CONFIG_MAPPING_HPP

namespace fastdem {

/// Mapping mode for height map origin behavior.
enum class MappingMode {
  LOCAL,  ///< Map origin follows robot (robot-centric)
  GLOBAL  ///< Map origin is fixed (world frame)
};

/// Height estimation algorithm type.
enum class EstimationType {
  Kalman,     ///< Recursive Bayesian (Kalman filter)
  Welford,    ///< Online mean (Welford's algorithm)
  P2Quantile  ///< Online quantile (P² algorithm)
};

namespace config {

/**
 * @brief Kalman filter parameters for height estimation.
 *
 * Note: measurement variance is computed by sensor_model, not configured here.
 */
struct Kalman {
  float min_variance =
      0.0001f;                 ///< Minimum variance (prevents over-confidence)
  float max_variance = 0.01f;  ///< Maximum variance (caps uncertainty)
  float process_noise = 0.0f;  ///< Process noise Q (prevents over-confidence)
};

/**
 * @brief P² quantile estimator parameters.
 *
 * Online quantile estimation using Jain & Chlamtac's P² algorithm.
 * Default markers at {0%, 16%, 50%, 84%, 100%} correspond to
 * {min, mean-1σ, median, mean+1σ, max} for normal distributions.
 */
struct P2Quantile {
  /// Target quantiles for the 5 markers (must be sorted, 0 to 1)
  float dn0 = 0.0f;   ///< 0th percentile (min)
  float dn1 = 0.16f;  ///< 16th percentile (≈ mean - 1σ)
  float dn2 = 0.50f;  ///< 50th percentile (median)
  float dn3 = 0.84f;  ///< 84th percentile (≈ mean + 1σ)
  float dn4 = 1.0f;   ///< 100th percentile (max)

  /// Which marker to use as elevation output (0-4, default: 3 = 84th
  /// percentile)
  int elevation_marker = 3;

  /// Maximum sample count for fading memory (0 = disabled, no forgetting)
  /// When count exceeds this value, count and marker positions are rescaled
  /// to maintain responsiveness to new data in dynamic environments.
  /// Recommended: 50-200 for dynamic scenes, 0 for static mapping.
  float max_sample_count = 0.0f;
};

/**
 * @brief Mapping configuration.
 */
struct Mapping {
  MappingMode mode = MappingMode::LOCAL;

  /// Elevation estimation algorithm
  EstimationType estimation_type = EstimationType::Kalman;
  Kalman kalman;
  P2Quantile p2;

  /// Sigma scale for elevation output: elevation = estimate + sigma_scale * σ
  /// (0 = center estimate, 1 ≈ 84th percentile). Welford/Kalman only.
  float sigma_scale = 0.0f;
};

}  // namespace config
}  // namespace fastdem

#endif  // FASTDEM_CONFIG_MAPPING_HPP
