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

#include "fastdem/config/postprocess.hpp"
#include "fastdem/elevation_map.hpp"

namespace fastdem {

/**
 * @brief Spatially fuses estimator-computed bounds using bilateral weighting.
 *
 * Takes per-cell upper_bound/lower_bound from the height estimator and
 * produces neighborhood-aware bounds that preserve edges while smoothing
 * uncertainty in flat areas.
 *
 * Input layers: upper_bound, lower_bound (from estimator computeBounds)
 * Output layers: upper_bound, lower_bound (overwritten)
 *
 * @param map Height map to process (must have upper_bound, lower_bound layers)
 * @param config Spatial fusion configuration
 */
void applyUncertaintyFusion(ElevationMap& map, const config::UncertaintyFusion& config);

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_UNCERTAINTY_FUSION_HPP
