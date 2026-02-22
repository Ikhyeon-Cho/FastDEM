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
  if (type == "kalman_filter") return EstimationType::Kalman;
  if (type == "p2_quantile") return EstimationType::P2Quantile;
  spdlog::warn("[Config] Unknown estimation type '{}', defaulting to kalman_filter",
               type);
  return EstimationType::Kalman;
}

MappingMode parseMappingMode(const std::string& mode) {
  if (mode == "local") return MappingMode::LOCAL;
  if (mode == "global") return MappingMode::GLOBAL;
  spdlog::warn("[Config] Unknown mapping mode '{}', defaulting to local", mode);
  return MappingMode::LOCAL;
}

SensorType parseSensorType(const std::string& type) {
  if (type == "lidar" || type == "laser") return SensorType::LiDAR;
  if (type == "rgbd") return SensorType::RGBD;
  if (type == "constant" || type == "none") return SensorType::Constant;
  spdlog::warn("[Config] Unknown sensor_model.type '{}', defaulting to LiDAR",
               type);
  return SensorType::LiDAR;
}

Config parse(const YAML::Node& root) {
  Config cfg;

  // Mapping (estimation parameters)
  if (auto n = root["mapping"]) {
    auto& m = cfg.mapping;
    std::string mode_str;
    load(n, "mode", mode_str);
    if (!mode_str.empty()) m.mode = parseMappingMode(mode_str);
    std::string estimation_str;
    load(n, "type", estimation_str);
    if (!estimation_str.empty())
      m.estimation_type = parseEstimationType(estimation_str);
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

  // Point filter
  if (auto n = root["point_filter"]) {
    load(n, "z_min", cfg.point_filter.z_min);
    load(n, "z_max", cfg.point_filter.z_max);
    load(n, "range_min", cfg.point_filter.range_min);
    load(n, "range_max", cfg.point_filter.range_max);
  }

  // Raycasting (log-odds ghost removal)
  if (auto n = root["raycasting"]) {
    load(n, "enabled", cfg.raycasting.enabled);
    load(n, "height_conflict_threshold", cfg.raycasting.height_conflict_threshold);
    load(n, "log_odds_observed", cfg.raycasting.log_odds_observed);
    load(n, "log_odds_ghost", cfg.raycasting.log_odds_ghost);
    load(n, "log_odds_max", cfg.raycasting.log_odds_max);
    load(n, "clear_threshold", cfg.raycasting.clear_threshold);
  }

  // Sensor model
  if (auto n = root["sensor_model"]) {
    std::string sensor_str;
    load(n, "type", sensor_str);
    if (!sensor_str.empty())
      cfg.sensor_model.type = parseSensorType(sensor_str);
    if (auto l = n["lidar"]) {
      load(l, "range_noise", cfg.sensor_model.lidar.range_noise);
      load(l, "angular_noise", cfg.sensor_model.lidar.angular_noise);
    }
    if (auto r = n["rgbd"]) {
      load(r, "normal_a", cfg.sensor_model.rgbd.normal_a);
      load(r, "normal_b", cfg.sensor_model.rgbd.normal_b);
      load(r, "normal_c", cfg.sensor_model.rgbd.normal_c);
      load(r, "lateral_factor", cfg.sensor_model.rgbd.lateral_factor);
    }
    if (auto c = n["constant"]) {
      load(c, "uncertainty", cfg.sensor_model.constant.uncertainty);
    }
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
    if (m.raycasting.height_conflict_threshold <= 0.0f) {
      spdlog::warn("[Config] raycasting.height_conflict_threshold ({}) must be > 0, "
                   "clamping to 0.05",
                   m.raycasting.height_conflict_threshold);
      m.raycasting.height_conflict_threshold = 0.05f;
    }
    if (m.raycasting.log_odds_observed <= 0.0f) {
      spdlog::warn("[Config] raycasting.log_odds_observed ({}) must be > 0, "
                   "clamping to 0.4",
                   m.raycasting.log_odds_observed);
      m.raycasting.log_odds_observed = 0.4f;
    }
    if (m.raycasting.log_odds_ghost <= 0.0f) {
      spdlog::warn("[Config] raycasting.log_odds_ghost ({}) must be > 0, "
                   "clamping to 0.2",
                   m.raycasting.log_odds_ghost);
      m.raycasting.log_odds_ghost = 0.2f;
    }
    if (m.raycasting.log_odds_max <= 0.0f) {
      spdlog::warn("[Config] raycasting.log_odds_max ({}) must be > 0, "
                   "clamping to 2.0",
                   m.raycasting.log_odds_max);
      m.raycasting.log_odds_max = 2.0f;
    }
    if (m.raycasting.clear_threshold >= 0.0f) {
      spdlog::warn("[Config] raycasting.clear_threshold ({}) must be < 0, "
                   "clamping to -1.0",
                   m.raycasting.clear_threshold);
      m.raycasting.clear_threshold = -1.0f;
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

  // P2 quantile markers must be in [0, 1] and monotonically non-decreasing
  auto& p2 = m.mapping.p2;
  float* dns[] = {&p2.dn0, &p2.dn1, &p2.dn2, &p2.dn3, &p2.dn4};
  for (int i = 0; i < 5; ++i) {
    if (*dns[i] < 0.0f || *dns[i] > 1.0f) {
      spdlog::warn("[Config] mapping.p2.dn{} ({}) out of [0, 1], clamping", i,
                   *dns[i]);
      *dns[i] = std::clamp(*dns[i], 0.0f, 1.0f);
    }
  }
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
  if (m.sensor_model.lidar.range_noise <= 0.0f) {
    spdlog::warn(
        "[Config] sensor.lidar.range_noise ({}) must be > 0, clamping to 0.02",
        m.sensor_model.lidar.range_noise);
    m.sensor_model.lidar.range_noise = 0.02f;
  }
  if (m.sensor_model.lidar.angular_noise < 0.0f) {
    spdlog::warn(
        "[Config] sensor.lidar.angular_noise ({}) must be >= 0, clamping to 0",
        m.sensor_model.lidar.angular_noise);
    m.sensor_model.lidar.angular_noise = 0.0f;
  }
  if (m.sensor_model.constant.uncertainty <= 0.0f) {
    spdlog::warn(
        "[Config] sensor.constant.uncertainty ({}) must be > 0, "
        "clamping to 0.1",
        m.sensor_model.constant.uncertainty);
    m.sensor_model.constant.uncertainty = 0.1f;
  }
  if (m.sensor_model.rgbd.normal_a < 0.0f) {
    spdlog::warn("[Config] sensor.rgbd.normal_a ({}) must be >= 0, clamping to 0",
                 m.sensor_model.rgbd.normal_a);
    m.sensor_model.rgbd.normal_a = 0.0f;
  }
  if (m.sensor_model.rgbd.normal_b < 0.0f) {
    spdlog::warn("[Config] sensor.rgbd.normal_b ({}) must be >= 0, clamping to 0",
                 m.sensor_model.rgbd.normal_b);
    m.sensor_model.rgbd.normal_b = 0.0f;
  }
  if (m.sensor_model.rgbd.normal_c < 0.0f) {
    spdlog::warn("[Config] sensor.rgbd.normal_c ({}) must be >= 0, clamping to 0",
                 m.sensor_model.rgbd.normal_c);
    m.sensor_model.rgbd.normal_c = 0.0f;
  }
  if (m.sensor_model.rgbd.lateral_factor < 0.0f) {
    spdlog::warn(
        "[Config] sensor.rgbd.lateral_factor ({}) must be >= 0, clamping to 0",
        m.sensor_model.rgbd.lateral_factor);
    m.sensor_model.rgbd.lateral_factor = 0.0f;
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
