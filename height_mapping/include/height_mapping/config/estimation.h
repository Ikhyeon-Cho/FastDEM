/*
 * estimation.h
 *
 * Height estimation algorithm configuration types.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_ESTIMATION_H
#define HEIGHT_MAPPING_CONFIG_ESTIMATION_H

#include <string>

namespace height_mapping::config {

/**
 * @brief Kalman filter parameters for height estimation.
 *
 * Note: measurement variance is computed by sensor_model, not configured here.
 */
struct Kalman {
  float min_variance = 0.0001f;  ///< Minimum variance (prevents over-confidence)
  float max_variance = 0.01f;    ///< Maximum variance (caps uncertainty)
  float process_noise = 0.0f;    ///< Process noise Q (prevents over-confidence)
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

  /// Which marker to use as elevation output (0-4, default: 3 = 84th percentile)
  int elevation_marker = 3;

  /// Maximum sample count for fading memory (0 = disabled, no forgetting)
  /// When count exceeds this value, count and marker positions are rescaled
  /// to maintain responsiveness to new data in dynamic environments.
  /// Recommended: 50-200 for dynamic scenes, 0 for static mapping.
  float max_sample_count = 0.0f;
};

/**
 * @brief Height estimation configuration.
 */
struct Estimation {
  /// Algorithm type: kalman_filter, welford, p2_quantile
  std::string type = "kalman_filter";
  Kalman kalman;
  P2Quantile p2;

  /// Fixed alpha for elevation = alpha * max_denoised + (1 - alpha) * state
  /// alpha = 0: use state only (pure estimation)
  /// alpha = 0.5: balanced (default)
  /// alpha = 0.9: conservative (trust max more)
  float alpha = 0.5f;
};

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_ESTIMATION_H
