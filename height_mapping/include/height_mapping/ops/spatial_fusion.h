/*
 * spatial_fusion.h
 *
 * Spatial uncertainty fusion using bilateral filter + weighted ECDF.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_OPS_SPATIAL_FUSION_H
#define HEIGHT_MAPPING_OPS_SPATIAL_FUSION_H

#include "height_mapping/config/spatial_fusion.h"
#include "height_mapping/core/map.h"

namespace height_mapping::ops {

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
void applySpatialFusion(HeightMap& map, const config::SpatialFusion& config);

}  // namespace height_mapping::ops

#endif  // HEIGHT_MAPPING_OPS_SPATIAL_FUSION_H
