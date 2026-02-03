// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * uncertainty_fusion.hpp
 *
 * Uncertainty fusion using bilateral filter + weighted ECDF.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_POSTPROCESS_UNCERTAINTY_FUSION_HPP
#define FASTDEM_POSTPROCESS_UNCERTAINTY_FUSION_HPP

#include "fastdem/config/uncertainty_fusion.hpp"
#include "fastdem/elevation_map.hpp"

namespace fastdem {

/**
 * @brief Computes spatially-fused uncertainty_range using bilateral weighting.
 *
 * Replaces the simple per-cell 4σ uncertainty with a neighborhood-aware
 * estimate that preserves edges while smoothing uncertainty in flat areas.
 *
 * Input layers: state (mean height), variance (Kalman covariance)
 * Output layer: uncertainty_range (overwrites existing 4σ value)
 *
 * @param map Height map to process (must have state, variance layers)
 * @param config Spatial fusion configuration
 */
void applyUncertaintyFusion(ElevationMap& map, const config::UncertaintyFusion& config);

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_UNCERTAINTY_FUSION_HPP
