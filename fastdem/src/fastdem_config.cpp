// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * config.cpp
 *
 * YAML configuration loading.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

#include "fastdem/config/fastdem.hpp"

namespace fastdem {
namespace {

template <typename T>
void load(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

MappingMode parseMode(const std::string& mode) {
  if (mode == "global") return MappingMode::GLOBAL;
  return MappingMode::LOCAL;
}

EstimationType parseEstimationType(const std::string& type) {
  if (type == "welford") return EstimationType::Welford;
  if (type == "p2_quantile") return EstimationType::P2Quantile;
  return EstimationType::Kalman;
}

RasterMethod parseRasterMethod(const std::string& method) {
  if (method == "min") return RasterMethod::Min;
  if (method == "mean") return RasterMethod::Mean;
  return RasterMethod::Max;
}

SensorType parseSensorType(const std::string& type) {
  if (type == "lidar" || type == "laser") return SensorType::LiDAR;
  if (type == "rgbd") return SensorType::RGBD;
  if (type == "constant" || type == "none") return SensorType::Constant;
  return SensorType::LiDAR;
}

Config parseConfig(const YAML::Node& root) {
  Config cfg;

  // --- CoreConfig ---

  // Mapping (mode + estimation)
  if (auto n = root["mapping"]) {
    auto& m = cfg.core.mapping;
    std::string mode;
    load(n, "mode", mode);
    if (!mode.empty()) m.mode = parseMode(mode);
    std::string estimation_str;
    load(n, "type", estimation_str);
    if (!estimation_str.empty())
      m.estimation_type = parseEstimationType(estimation_str);
    load(n, "sigma_scale", m.sigma_scale);
    if (auto k = n["kalman"]) {
      load(k, "min_variance", m.kalman.min_variance);
      load(k, "max_variance", m.kalman.max_variance);
      load(k, "process_noise", m.kalman.process_noise);
    }
    if (auto p = n["p2"]) {
      load(p, "dn0", m.p2.dn0);
      load(p, "dn1", m.p2.dn1);
      load(p, "dn2", m.p2.dn2);
      load(p, "dn3", m.p2.dn3);
      load(p, "dn4", m.p2.dn4);
      load(p, "elevation_marker", m.p2.elevation_marker);
      load(p, "max_sample_count", m.p2.max_sample_count);
    }
  }

  // Scan filter
  if (auto n = root["scan_filter"]) {
    load(n, "z_min", cfg.core.scan_filter.z_min);
    load(n, "z_max", cfg.core.scan_filter.z_max);
    load(n, "range_min", cfg.core.scan_filter.range_min);
    load(n, "range_max", cfg.core.scan_filter.range_max);
  }

  // Rasterization
  if (auto n = root["rasterization"]) {
    std::string method_str;
    load(n, "method", method_str);
    if (!method_str.empty())
      cfg.core.rasterization.method = parseRasterMethod(method_str);
  }

  // Raycasting (temporal voting for ghost removal)
  if (auto n = root["raycasting"]) {
    load(n, "enabled", cfg.core.raycasting.enabled);
    load(n, "endpoint_margin", cfg.core.raycasting.endpoint_margin);
    load(n, "ray_height_margin", cfg.core.raycasting.ray_height_margin);
    load(n, "dynamic_height_threshold",
         cfg.core.raycasting.dynamic_height_threshold);
    load(n, "vote_threshold", cfg.core.raycasting.vote_threshold);
  }

  // Sensor model
  if (auto n = root["sensor"]) {
    std::string sensor_str;
    load(n, "type", sensor_str);
    if (!sensor_str.empty())
      cfg.core.sensor.type = parseSensorType(sensor_str);
    load(n, "range_noise", cfg.core.sensor.range_noise);
    load(n, "angular_noise", cfg.core.sensor.angular_noise);
    load(n, "normal_a", cfg.core.sensor.normal_a);
    load(n, "normal_b", cfg.core.sensor.normal_b);
    load(n, "normal_c", cfg.core.sensor.normal_c);
    load(n, "lateral_factor", cfg.core.sensor.lateral_factor);
    load(n, "constant_uncertainty", cfg.core.sensor.constant_uncertainty);
  }

  // Uncertainty fusion (bilateral filter + weighted ECDF)
  if (auto n = root["uncertainty_fusion"]) {
    load(n, "enabled", cfg.core.uncertainty_fusion.enabled);
    load(n, "search_radius", cfg.core.uncertainty_fusion.search_radius);
    load(n, "spatial_sigma", cfg.core.uncertainty_fusion.spatial_sigma);
    load(n, "quantile_lower", cfg.core.uncertainty_fusion.quantile_lower);
    load(n, "quantile_upper", cfg.core.uncertainty_fusion.quantile_upper);
    load(n, "min_valid_neighbors",
         cfg.core.uncertainty_fusion.min_valid_neighbors);
  }

  // --- PostProcessConfig ---

  // Inpainting
  if (auto n = root["inpainting"]) {
    load(n, "enabled", cfg.postprocess.inpainting.enabled);
    load(n, "max_iterations", cfg.postprocess.inpainting.max_iterations);
    load(n, "min_valid_neighbors",
         cfg.postprocess.inpainting.min_valid_neighbors);
  }

  // Feature extraction (PCA-based terrain features)
  if (auto n = root["feature_extraction"]) {
    load(n, "enabled", cfg.postprocess.feature_extraction.enabled);
    load(n, "analysis_radius",
         cfg.postprocess.feature_extraction.analysis_radius);
    load(n, "min_valid_neighbors",
         cfg.postprocess.feature_extraction.min_valid_neighbors);
  }

  return cfg;
}

void validateConfig(Config& cfg) {
  auto& m = cfg.core;
  auto& pp = cfg.postprocess;

  // --- Fatal: invalid ranges that break the pipeline ---
  if (m.scan_filter.z_min >= m.scan_filter.z_max) {
    throw std::invalid_argument(
        "scan_filter: z_min (" + std::to_string(m.scan_filter.z_min) +
        ") >= z_max (" + std::to_string(m.scan_filter.z_max) + ")");
  }
  if (m.scan_filter.range_min >= m.scan_filter.range_max) {
    throw std::invalid_argument(
        "scan_filter: range_min (" + std::to_string(m.scan_filter.range_min) +
        ") >= range_max (" + std::to_string(m.scan_filter.range_max) + ")");
  }
  if (m.mapping.kalman.min_variance >= m.mapping.kalman.max_variance) {
    throw std::invalid_argument(
        "mapping.kalman: min_variance (" +
        std::to_string(m.mapping.kalman.min_variance) + ") >= max_variance (" +
        std::to_string(m.mapping.kalman.max_variance) + ")");
  }
  if (m.uncertainty_fusion.quantile_lower >
      m.uncertainty_fusion.quantile_upper) {
    throw std::invalid_argument(
        "uncertainty_fusion: quantile_lower (" +
        std::to_string(m.uncertainty_fusion.quantile_lower) +
        ") > quantile_upper (" +
        std::to_string(m.uncertainty_fusion.quantile_upper) + ")");
  }

  // --- Non-fatal: warn and clamp ---
  auto warn_clamp = [](const std::string& name, auto& val, auto lo, auto hi) {
    if (val < lo || val > hi) {
      spdlog::warn("[Config] {} ({}) out of range [{}, {}], clamping", name, val,
                   lo, hi);
      val = std::clamp(val, static_cast<decltype(val)>(lo),
                       static_cast<decltype(val)>(hi));
    }
  };

  warn_clamp("scan_filter.range_min", m.scan_filter.range_min, 0.0f,
             m.scan_filter.range_max);

  if (m.raycasting.enabled) {
    warn_clamp("raycasting.endpoint_margin", m.raycasting.endpoint_margin, 0,
               100);
    if (m.raycasting.ray_height_margin <= 0.0f) {
      spdlog::warn("[Config] raycasting.ray_height_margin ({}) must be > 0, "
                   "clamping to 0.05",
                   m.raycasting.ray_height_margin);
      m.raycasting.ray_height_margin = 0.05f;
    }
    if (m.raycasting.vote_threshold <= 0) {
      spdlog::warn(
          "[Config] raycasting.vote_threshold ({}) must be > 0, clamping to 1",
          m.raycasting.vote_threshold);
      m.raycasting.vote_threshold = 1;
    }
  }

  if (m.mapping.kalman.min_variance <= 0.0f) {
    spdlog::warn(
        "[Config] estimation.kalman.min_variance ({}) must be > 0, "
        "clamping to 0.0001",
        m.mapping.kalman.min_variance);
    m.mapping.kalman.min_variance = 0.0001f;
  }
  if (m.mapping.kalman.process_noise < 0.0f) {
    spdlog::warn(
        "[Config] estimation.kalman.process_noise ({}) must be >= 0, "
        "clamping to 0",
        m.mapping.kalman.process_noise);
    m.mapping.kalman.process_noise = 0.0f;
  }
  warn_clamp("mapping.p2.elevation_marker", m.mapping.p2.elevation_marker,
             0, 4);

  // P2 quantile markers must be monotonically non-decreasing
  const auto& p2 = m.mapping.p2;
  if (p2.dn0 > p2.dn1 || p2.dn1 > p2.dn2 || p2.dn2 > p2.dn3 ||
      p2.dn3 > p2.dn4) {
    throw std::invalid_argument(
        "mapping.p2: markers must be sorted (dn0 <= dn1 <= dn2 <= dn3 <= "
        "dn4), got {" +
        std::to_string(p2.dn0) + ", " + std::to_string(p2.dn1) + ", " +
        std::to_string(p2.dn2) + ", " + std::to_string(p2.dn3) + ", " +
        std::to_string(p2.dn4) + "}");
  }

  // Sensor model parameters must be positive
  if (m.sensor.range_noise <= 0.0f) {
    spdlog::warn(
        "[Config] sensor.range_noise ({}) must be > 0, clamping to 0.02",
        m.sensor.range_noise);
    m.sensor.range_noise = 0.02f;
  }
  if (m.sensor.angular_noise < 0.0f) {
    spdlog::warn(
        "[Config] sensor.angular_noise ({}) must be >= 0, clamping to 0",
        m.sensor.angular_noise);
    m.sensor.angular_noise = 0.0f;
  }
  if (m.sensor.constant_uncertainty <= 0.0f) {
    spdlog::warn(
        "[Config] sensor.constant_uncertainty ({}) must be > 0, "
        "clamping to 0.1",
        m.sensor.constant_uncertainty);
    m.sensor.constant_uncertainty = 0.1f;
  }

  if (m.uncertainty_fusion.enabled) {
    if (m.uncertainty_fusion.search_radius <= 0.0f) {
      spdlog::warn("[Config] uncertainty_fusion.search_radius ({}) must be > 0, "
                   "clamping to 0.15",
                   m.uncertainty_fusion.search_radius);
      m.uncertainty_fusion.search_radius = 0.15f;
    }
    if (m.uncertainty_fusion.spatial_sigma <= 0.0f) {
      spdlog::warn("[Config] uncertainty_fusion.spatial_sigma ({}) must be > 0, "
                   "clamping to 0.05",
                   m.uncertainty_fusion.spatial_sigma);
      m.uncertainty_fusion.spatial_sigma = 0.05f;
    }
  }

  // Post-processing validation
  if (pp.inpainting.enabled) {
    if (pp.inpainting.max_iterations < 1) {
      spdlog::warn(
          "[Config] inpainting.max_iterations ({}) must be >= 1, clamping to 1",
          pp.inpainting.max_iterations);
      pp.inpainting.max_iterations = 1;
    }
    warn_clamp("inpainting.min_valid_neighbors",
               pp.inpainting.min_valid_neighbors, 1, 8);
  }

  if (pp.feature_extraction.enabled) {
    if (pp.feature_extraction.analysis_radius <= 0.0f) {
      spdlog::warn(
          "[Config] feature_extraction.analysis_radius ({}) must be > 0, "
          "clamping to 0.3",
          pp.feature_extraction.analysis_radius);
      pp.feature_extraction.analysis_radius = 0.3f;
    }
    warn_clamp("feature_extraction.min_valid_neighbors",
               pp.feature_extraction.min_valid_neighbors, 4, 100);
  }
}

}  // namespace

Config loadConfig(const std::string& path) {
  try {
    auto cfg = parseConfig(YAML::LoadFile(path));
    validateConfig(cfg);
    return cfg;
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load config: " + path + " - " +
                             e.what());
  }
}

}  // namespace fastdem
