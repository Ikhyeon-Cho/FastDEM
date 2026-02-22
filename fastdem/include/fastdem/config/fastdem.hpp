// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_CONFIG_FASTDEM_HPP
#define FASTDEM_CONFIG_FASTDEM_HPP

#include <limits>
#include <string>

namespace YAML {
class Node;
}

#include "fastdem/config/mapping.hpp"
#include "fastdem/config/postprocess.hpp"
#include "fastdem/config/sensor_model.hpp"

namespace fastdem {

namespace config {

/// Point filtering bounds (z-range and distance).
struct PointFilter {
  float z_min = -std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::max();
  float range_min = 0.0f;
  float range_max = std::numeric_limits<float>::max();
};

}  // namespace config

/// Pipeline configuration for FastDEM.
struct Config {
  config::PointFilter point_filter;
  config::SensorModel sensor_model;
  config::Mapping mapping;
  config::Raycasting raycasting;
};

Config parseConfig(const YAML::Node& root);
Config loadConfig(const std::string& path);

}  // namespace fastdem

#endif  // FASTDEM_CONFIG_FASTDEM_HPP
