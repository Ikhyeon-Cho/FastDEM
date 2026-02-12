// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_CONFIG_FEATURE_EXTRACTION_HPP
#define FASTDEM_CONFIG_FEATURE_EXTRACTION_HPP

namespace fastdem::config {

struct FeatureExtraction {
  bool enabled = false;
  float analysis_radius = 0.3f;  // [m] PCA neighbor radius
  int min_valid_neighbors = 4;   // minimum neighbors for valid PCA
};

}  // namespace fastdem::config

#endif  // FASTDEM_CONFIG_FEATURE_EXTRACTION_HPP
