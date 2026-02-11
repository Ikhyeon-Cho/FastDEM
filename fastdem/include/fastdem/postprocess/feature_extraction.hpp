// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_POSTPROCESS_FEATURE_EXTRACTION_HPP
#define FASTDEM_POSTPROCESS_FEATURE_EXTRACTION_HPP

#include "fastdem/elevation_map.hpp"

namespace fastdem {

/// @brief Extract terrain features from elevation map using local PCA.
///
/// For each measured cell, gathers 3D neighbor positions within
/// analysis_radius, computes PCA on the local surface patch, and
/// derives geometric features.
///
/// Input layer:  elevation
/// Output layers: step, slope, roughness, curvature, normal_x, normal_y,
/// normal_z
void applyFeatureExtraction(ElevationMap& map,
                            float analysis_radius = 0.3f,
                            int min_valid_neighbors = 4);

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_FEATURE_EXTRACTION_HPP
