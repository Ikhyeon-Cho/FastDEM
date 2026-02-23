// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_CONFIG_POSTPROCESS_HPP
#define FASTDEM_CONFIG_POSTPROCESS_HPP

#include <string>

namespace YAML {
class Node;
}

namespace fastdem::config {

/// Ghost obstacle removal via log-odds free-space accumulation.
struct Raycasting {
  bool enabled = false;
  float height_conflict_threshold = 0.05f;  ///< Min height diff for conflict [m]
  float log_odds_observed = 0.4f;     ///< Evidence for cell being present (measured)
  float log_odds_ghost = 0.2f;        ///< Evidence for cell being empty (ray through)
  float log_odds_max = 2.0f;          ///< Upper clamp (prevents over-confidence)
  float clear_threshold = -1.0f;      ///< Clear cell when logodds falls below this
};

/// Iterative neighbor averaging for NaN holes.
struct Inpainting {
  bool enabled = false;
  int max_iterations = 3;
  int min_valid_neighbors = 2;
};

/// Spatial fusion of estimator bounds (inverse-range weighted ECDF).
struct UncertaintyFusion {
  bool enabled = false;
  float search_radius = 0.15f;   ///< Neighbor search radius [m]
  float spatial_sigma = 0.05f;   ///< Gaussian distance decay [m]
  float quantile_lower = 0.01f;  ///< Lower quantile for bound
  float quantile_upper = 0.99f;  ///< Upper quantile for bound
  int min_valid_neighbors = 3;
};

/// Local PCA-based terrain feature extraction.
struct FeatureExtraction {
  bool enabled = false;
  float analysis_radius = 0.3f;      ///< PCA neighbor radius [m]
  int min_valid_neighbors = 4;
  float step_lower_percentile = 0.05f;  ///< Lower percentile for step [0,1]
  float step_upper_percentile = 0.95f;  ///< Upper percentile for step [0,1]
};

/// Post-processing configuration aggregate.
struct PostProcess {
  Inpainting inpainting;
  UncertaintyFusion uncertainty_fusion;
  FeatureExtraction feature_extraction;
};

PostProcess parsePostProcess(const YAML::Node& root);
PostProcess loadPostProcess(const std::string& path);

}  // namespace fastdem::config

#endif  // FASTDEM_CONFIG_POSTPROCESS_HPP
