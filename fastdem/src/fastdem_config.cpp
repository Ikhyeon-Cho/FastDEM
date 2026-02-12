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
namespace detail {

template <typename T>
void load(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
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

Config parse(const YAML::Node& root) {
  Config cfg;

  // Mapping (estimation parameters)
  if (auto n = root["mapping"]) {
    auto& m = cfg.mapping;
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

  // Sensor model
  if (auto n = root["sensor_model"]) {
    std::string sensor_str;
    load(n, "type", sensor_str);
    if (!sensor_str.empty())
      cfg.sensor_model.type = parseSensorType(sensor_str);
    load(n, "range_noise", cfg.sensor_model.range_noise);
    load(n, "angular_noise", cfg.sensor_model.angular_noise);
    load(n, "normal_a", cfg.sensor_model.normal_a);
    load(n, "normal_b", cfg.sensor_model.normal_b);
    load(n, "normal_c", cfg.sensor_model.normal_c);
    load(n, "lateral_factor", cfg.sensor_model.lateral_factor);
    load(n, "constant_uncertainty", cfg.sensor_model.constant_uncertainty);
  }

  // Uncertainty fusion (bilateral filter + weighted ECDF)
  if (auto n = root["uncertainty_fusion"]) {
    load(n, "enabled", cfg.uncertainty_fusion.enabled);
    load(n, "search_radius", cfg.uncertainty_fusion.search_radius);
    load(n, "spatial_sigma", cfg.uncertainty_fusion.spatial_sigma);
    load(n, "quantile_lower", cfg.uncertainty_fusion.quantile_lower);
    load(n, "quantile_upper", cfg.uncertainty_fusion.quantile_upper);
    load(n, "min_valid_neighbors",
         cfg.uncertainty_fusion.min_valid_neighbors);
  }

  return cfg;
}

void validate(Config& cfg) {
  auto& m = cfg;

  // --- Fatal: invalid ranges that break the pipeline ---
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
  if (m.sensor_model.range_noise <= 0.0f) {
    spdlog::warn(
        "[Config] sensor.range_noise ({}) must be > 0, clamping to 0.02",
        m.sensor_model.range_noise);
    m.sensor_model.range_noise = 0.02f;
  }
  if (m.sensor_model.angular_noise < 0.0f) {
    spdlog::warn(
        "[Config] sensor.angular_noise ({}) must be >= 0, clamping to 0",
        m.sensor_model.angular_noise);
    m.sensor_model.angular_noise = 0.0f;
  }
  if (m.sensor_model.constant_uncertainty <= 0.0f) {
    spdlog::warn(
        "[Config] sensor.constant_uncertainty ({}) must be > 0, "
        "clamping to 0.1",
        m.sensor_model.constant_uncertainty);
    m.sensor_model.constant_uncertainty = 0.1f;
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

}

}  // namespace detail

Config parseConfig(const YAML::Node& root) {
  auto cfg = detail::parse(root);
  detail::validate(cfg);
  return cfg;
}

Config loadConfig(const std::string& path) {
  try {
    return parseConfig(YAML::LoadFile(path));
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load config: " + path + " - " +
                             e.what());
  }
}

}  // namespace fastdem
