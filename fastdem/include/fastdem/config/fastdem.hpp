// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * fastdem.hpp
 *
 * Configuration for FastDEM.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_CONFIG_FASTDEM_HPP
#define FASTDEM_CONFIG_FASTDEM_HPP

#include <string>

namespace YAML {
class Node;
}

#include "fastdem/config/mapping.hpp"
#include "fastdem/config/point_filter.hpp"
#include "fastdem/config/rasterization.hpp"
#include "fastdem/config/raycasting.hpp"
#include "fastdem/config/sensor_model.hpp"
#include "fastdem/config/uncertainty_fusion.hpp"

namespace fastdem {

/// Algorithm configuration for the FastDEM pipeline.
struct Config {
  config::PointFilter point_filter;
  config::SensorModel sensor_model;
  config::Rasterization rasterization;
  config::Mapping mapping;
  config::Raycasting raycasting;
  config::UncertaintyFusion uncertainty_fusion;
};

/// Parse pipeline configuration from a YAML node.
Config parseConfig(const YAML::Node& root);

/// Load pipeline configuration from a YAML file.
Config loadConfig(const std::string& path);

}  // namespace fastdem

#endif  // FASTDEM_CONFIG_FASTDEM_HPP
