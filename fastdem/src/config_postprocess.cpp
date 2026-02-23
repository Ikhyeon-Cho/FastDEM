// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>

#include "fastdem/config/postprocess.hpp"

namespace fastdem::config {
namespace detail {

template <typename T>
void load(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

void validate(PostProcess& cfg) {
  auto warn_clamp_positive = [](const std::string& name, float& val,
                                float fallback) {
    if (val <= 0.0f) {
      spdlog::warn("[PostProcess] {} ({}) must be > 0, clamping to {}", name,
                   val, fallback);
      val = fallback;
    }
  };

  auto warn_clamp_min = [](const std::string& name, int& val, int lo) {
    if (val < lo) {
      spdlog::warn("[PostProcess] {} ({}) must be >= {}, clamping", name, val,
                   lo);
      val = lo;
    }
  };

  // Inpainting
  warn_clamp_min("inpainting.max_iterations", cfg.inpainting.max_iterations, 1);
  warn_clamp_min("inpainting.min_valid_neighbors",
                 cfg.inpainting.min_valid_neighbors, 1);

  // Uncertainty fusion
  warn_clamp_positive("uncertainty_fusion.search_radius",
                      cfg.uncertainty_fusion.search_radius, 0.15f);
  warn_clamp_positive("uncertainty_fusion.spatial_sigma",
                      cfg.uncertainty_fusion.spatial_sigma, 0.05f);
  warn_clamp_min("uncertainty_fusion.min_valid_neighbors",
                 cfg.uncertainty_fusion.min_valid_neighbors, 1);

  auto& ql = cfg.uncertainty_fusion.quantile_lower;
  auto& qu = cfg.uncertainty_fusion.quantile_upper;
  ql = std::clamp(ql, 0.0f, 1.0f);
  qu = std::clamp(qu, 0.0f, 1.0f);
  if (ql >= qu) {
    spdlog::warn(
        "[PostProcess] uncertainty_fusion.quantile_lower ({}) >= "
        "quantile_upper ({}), resetting to defaults",
        ql, qu);
    ql = 0.01f;
    qu = 0.99f;
  }

  // Feature extraction
  warn_clamp_positive("feature_extraction.analysis_radius",
                      cfg.feature_extraction.analysis_radius, 0.3f);
  warn_clamp_min("feature_extraction.min_valid_neighbors",
                 cfg.feature_extraction.min_valid_neighbors, 3);

  auto& sl = cfg.feature_extraction.step_lower_percentile;
  auto& su = cfg.feature_extraction.step_upper_percentile;
  sl = std::clamp(sl, 0.0f, 1.0f);
  su = std::clamp(su, 0.0f, 1.0f);
  if (sl >= su) {
    spdlog::warn(
        "[PostProcess] feature_extraction.step_lower_percentile ({}) >= "
        "step_upper_percentile ({}), resetting to defaults",
        sl, su);
    sl = 0.05f;
    su = 0.95f;
  }
}

}  // namespace detail

PostProcess parsePostProcess(const YAML::Node& root) {
  PostProcess cfg;

  if (auto n = root["inpainting"]) {
    detail::load(n, "enabled", cfg.inpainting.enabled);
    detail::load(n, "max_iterations", cfg.inpainting.max_iterations);
    detail::load(n, "min_valid_neighbors", cfg.inpainting.min_valid_neighbors);
  }

  if (auto n = root["uncertainty_fusion"]) {
    detail::load(n, "enabled", cfg.uncertainty_fusion.enabled);
    detail::load(n, "search_radius", cfg.uncertainty_fusion.search_radius);
    detail::load(n, "spatial_sigma", cfg.uncertainty_fusion.spatial_sigma);
    detail::load(n, "quantile_lower", cfg.uncertainty_fusion.quantile_lower);
    detail::load(n, "quantile_upper", cfg.uncertainty_fusion.quantile_upper);
    detail::load(n, "min_valid_neighbors",
                 cfg.uncertainty_fusion.min_valid_neighbors);
  }

  if (auto n = root["feature_extraction"]) {
    detail::load(n, "enabled", cfg.feature_extraction.enabled);
    detail::load(n, "analysis_radius", cfg.feature_extraction.analysis_radius);
    detail::load(n, "min_valid_neighbors",
                 cfg.feature_extraction.min_valid_neighbors);
    detail::load(n, "step_lower_percentile",
                 cfg.feature_extraction.step_lower_percentile);
    detail::load(n, "step_upper_percentile",
                 cfg.feature_extraction.step_upper_percentile);
  }

  detail::validate(cfg);
  return cfg;
}

PostProcess loadPostProcess(const std::string& path) {
  try {
    return parsePostProcess(YAML::LoadFile(path));
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load postprocess config: " + path +
                             " - " + e.what());
  }
}

}  // namespace fastdem::config
