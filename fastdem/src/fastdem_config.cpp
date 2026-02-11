// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * config.cpp
 *
 * YAML configuration loading for MappingConfig.
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

MappingConfig parseConfig(const YAML::Node& root) {
  MappingConfig cfg;

  // Mapping (mode + estimation)
  if (auto n = root["mapping"]) {
    auto& m = cfg.mapping;
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
    load(n, "z_min", cfg.scan_filter.z_min);
    load(n, "z_max", cfg.scan_filter.z_max);
    load(n, "range_min", cfg.scan_filter.range_min);
    load(n, "range_max", cfg.scan_filter.range_max);
  }

  // Rasterization
  if (auto n = root["rasterization"]) {
    std::string method_str;
    load(n, "method", method_str);
    if (!method_str.empty())
      cfg.rasterization.method = parseRasterMethod(method_str);
  }

  // Raycasting (temporal voting for ghost removal)
  if (auto n = root["raycasting"]) {
    load(n, "enabled", cfg.raycasting.enabled);
    load(n, "endpoint_margin", cfg.raycasting.endpoint_margin);
    load(n, "ray_height_margin", cfg.raycasting.ray_height_margin);
    load(n, "dynamic_height_threshold",
         cfg.raycasting.dynamic_height_threshold);
    load(n, "vote_threshold", cfg.raycasting.vote_threshold);
  }

  // Inpainting
  if (auto n = root["inpainting"]) {
    load(n, "enabled", cfg.inpainting.enabled);
    load(n, "max_iterations", cfg.inpainting.max_iterations);
    load(n, "min_valid_neighbors", cfg.inpainting.min_valid_neighbors);
  }

  // Sensor model
  if (auto n = root["sensor"]) {
    std::string sensor_str;
    load(n, "type", sensor_str);
    if (!sensor_str.empty())
      cfg.sensor.type = parseSensorType(sensor_str);
    // LiDAR parameters
    load(n, "range_noise", cfg.sensor.range_noise);
    load(n, "angular_noise", cfg.sensor.angular_noise);
    // RGB-D parameters
    load(n, "normal_a", cfg.sensor.normal_a);
    load(n, "normal_b", cfg.sensor.normal_b);
    load(n, "normal_c", cfg.sensor.normal_c);
    load(n, "lateral_factor", cfg.sensor.lateral_factor);
    // Constant model parameter
    load(n, "constant_uncertainty", cfg.sensor.constant_uncertainty);
  }

  // Feature extraction (PCA-based terrain features)
  if (auto n = root["feature_extraction"]) {
    load(n, "enabled", cfg.feature_extraction.enabled);
    load(n, "analysis_radius", cfg.feature_extraction.analysis_radius);
    load(n, "min_valid_neighbors", cfg.feature_extraction.min_valid_neighbors);
  }

  // Spatial fusion (bilateral filter + weighted ECDF)
  if (auto n = root["uncertainty_fusion"]) {
    load(n, "enabled", cfg.uncertainty_fusion.enabled);
    load(n, "search_radius", cfg.uncertainty_fusion.search_radius);
    load(n, "spatial_sigma", cfg.uncertainty_fusion.spatial_sigma);
    load(n, "quantile_lower", cfg.uncertainty_fusion.quantile_lower);
    load(n, "quantile_upper", cfg.uncertainty_fusion.quantile_upper);
    load(n, "min_valid_neighbors", cfg.uncertainty_fusion.min_valid_neighbors);
  }

  return cfg;
}

void validateConfig(MappingConfig& cfg) {
  // --- Fatal: invalid ranges that break the pipeline ---
  if (cfg.scan_filter.z_min >= cfg.scan_filter.z_max) {
    throw std::invalid_argument(
        "scan_filter: z_min (" + std::to_string(cfg.scan_filter.z_min) +
        ") >= z_max (" + std::to_string(cfg.scan_filter.z_max) + ")");
  }
  if (cfg.scan_filter.range_min >= cfg.scan_filter.range_max) {
    throw std::invalid_argument(
        "scan_filter: range_min (" + std::to_string(cfg.scan_filter.range_min) +
        ") >= range_max (" + std::to_string(cfg.scan_filter.range_max) + ")");
  }
  if (cfg.mapping.kalman.min_variance >= cfg.mapping.kalman.max_variance) {
    throw std::invalid_argument(
        "mapping.kalman: min_variance (" +
        std::to_string(cfg.mapping.kalman.min_variance) + ") >= max_variance (" +
        std::to_string(cfg.mapping.kalman.max_variance) + ")");
  }
  if (cfg.uncertainty_fusion.quantile_lower >
      cfg.uncertainty_fusion.quantile_upper) {
    throw std::invalid_argument(
        "uncertainty_fusion: quantile_lower (" +
        std::to_string(cfg.uncertainty_fusion.quantile_lower) +
        ") > quantile_upper (" +
        std::to_string(cfg.uncertainty_fusion.quantile_upper) + ")");
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

  warn_clamp("scan_filter.range_min", cfg.scan_filter.range_min, 0.0f,
             cfg.scan_filter.range_max);

  if (cfg.raycasting.enabled) {
    warn_clamp("raycasting.endpoint_margin", cfg.raycasting.endpoint_margin, 0,
               100);
    if (cfg.raycasting.ray_height_margin <= 0.0f) {
      spdlog::warn("[Config] raycasting.ray_height_margin ({}) must be > 0, "
                   "clamping to 0.05",
                   cfg.raycasting.ray_height_margin);
      cfg.raycasting.ray_height_margin = 0.05f;
    }
    if (cfg.raycasting.vote_threshold <= 0) {
      spdlog::warn(
          "[Config] raycasting.vote_threshold ({}) must be > 0, clamping to 1",
          cfg.raycasting.vote_threshold);
      cfg.raycasting.vote_threshold = 1;
    }
  }

  if (cfg.inpainting.enabled) {
    if (cfg.inpainting.max_iterations < 1) {
      spdlog::warn(
          "[Config] inpainting.max_iterations ({}) must be >= 1, clamping to 1",
          cfg.inpainting.max_iterations);
      cfg.inpainting.max_iterations = 1;
    }
    warn_clamp("inpainting.min_valid_neighbors",
               cfg.inpainting.min_valid_neighbors, 1, 8);
  }

  if (cfg.mapping.kalman.min_variance <= 0.0f) {
    spdlog::warn(
        "[Config] estimation.kalman.min_variance ({}) must be > 0, "
        "clamping to 0.0001",
        cfg.mapping.kalman.min_variance);
    cfg.mapping.kalman.min_variance = 0.0001f;
  }
  if (cfg.mapping.kalman.process_noise < 0.0f) {
    spdlog::warn(
        "[Config] estimation.kalman.process_noise ({}) must be >= 0, "
        "clamping to 0",
        cfg.mapping.kalman.process_noise);
    cfg.mapping.kalman.process_noise = 0.0f;
  }
  warn_clamp("mapping.p2.elevation_marker", cfg.mapping.p2.elevation_marker,
             0, 4);

  // P2 quantile markers must be monotonically non-decreasing
  const auto& p2 = cfg.mapping.p2;
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
  if (cfg.sensor.range_noise <= 0.0f) {
    spdlog::warn(
        "[Config] sensor.range_noise ({}) must be > 0, clamping to 0.02",
        cfg.sensor.range_noise);
    cfg.sensor.range_noise = 0.02f;
  }
  if (cfg.sensor.angular_noise < 0.0f) {
    spdlog::warn(
        "[Config] sensor.angular_noise ({}) must be >= 0, clamping to 0",
        cfg.sensor.angular_noise);
    cfg.sensor.angular_noise = 0.0f;
  }
  if (cfg.sensor.constant_uncertainty <= 0.0f) {
    spdlog::warn(
        "[Config] sensor.constant_uncertainty ({}) must be > 0, "
        "clamping to 0.1",
        cfg.sensor.constant_uncertainty);
    cfg.sensor.constant_uncertainty = 0.1f;
  }

  if (cfg.feature_extraction.enabled) {
    if (cfg.feature_extraction.analysis_radius <= 0.0f) {
      spdlog::warn(
          "[Config] feature_extraction.analysis_radius ({}) must be > 0, "
          "clamping to 0.3",
          cfg.feature_extraction.analysis_radius);
      cfg.feature_extraction.analysis_radius = 0.3f;
    }
    warn_clamp("feature_extraction.min_valid_neighbors",
               cfg.feature_extraction.min_valid_neighbors, 4, 100);
  }

  if (cfg.uncertainty_fusion.enabled) {
    if (cfg.uncertainty_fusion.search_radius <= 0.0f) {
      spdlog::warn("[Config] uncertainty_fusion.search_radius ({}) must be > 0, "
                   "clamping to 0.15",
                   cfg.uncertainty_fusion.search_radius);
      cfg.uncertainty_fusion.search_radius = 0.15f;
    }
    if (cfg.uncertainty_fusion.spatial_sigma <= 0.0f) {
      spdlog::warn("[Config] uncertainty_fusion.spatial_sigma ({}) must be > 0, "
                   "clamping to 0.05",
                   cfg.uncertainty_fusion.spatial_sigma);
      cfg.uncertainty_fusion.spatial_sigma = 0.05f;
    }
  }
}

}  // namespace

MappingConfig MappingConfig::load(const std::string& path) {
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
