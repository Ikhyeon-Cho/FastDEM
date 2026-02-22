// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_ROS_PARAMETERS_HPP
#define FASTDEM_ROS_PARAMETERS_HPP

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <fastdem/config/fastdem.hpp>
#include <fastdem/config/postprocess.hpp>
#include <string>

namespace fastdem::ros2 {

/// All configuration the node needs, parsed from a single YAML file.
struct NodeConfig {
  std::string logger_level{"info"};

  struct Topics {
    std::string input_scan{"/points"};
    double publish_rate{10.0};
    double post_process_rate{2.0};
  } topics;

  struct TF {
    std::string base_frame{"base_link"};
    std::string map_frame{"map"};
    double max_wait_time = 0.1;
    double max_stale_time = 0.1;
  } tf;

  fastdem::Config pipeline;

  struct {
    double width{15.0};
    double height{15.0};
    double resolution{0.1};
  } map;

  config::PostProcess postprocess;

  /// Parse everything from a single YAML file.
  static NodeConfig load(const std::string& config_path) {
    if (config_path.empty()) {
      throw std::invalid_argument("config_file path is empty");
    }
    auto yaml = YAML::LoadFile(config_path);
    NodeConfig cfg;

    auto read = [](const YAML::Node& n, const std::string& key, auto& val) {
      if (n[key]) val = n[key].as<std::decay_t<decltype(val)>>();
    };

    // ROS transport
    if (auto n = yaml["topics"]) {
      read(n, "input_scan", cfg.topics.input_scan);
      read(n, "publish_rate", cfg.topics.publish_rate);
      read(n, "post_process_rate", cfg.topics.post_process_rate);
    }
    if (auto n = yaml["tf"]) {
      read(n, "base_frame", cfg.tf.base_frame);
      read(n, "map_frame", cfg.tf.map_frame);
      read(n, "max_wait_time", cfg.tf.max_wait_time);
      read(n, "max_stale_time", cfg.tf.max_stale_time);
    }
    if (auto n = yaml["logger"]) {
      read(n, "level", cfg.logger_level);
    }

    // Map geometry
    if (auto n = yaml["map"]) {
      read(n, "width", cfg.map.width);
      read(n, "height", cfg.map.height);
      read(n, "resolution", cfg.map.resolution);
    }

    // Library configs (parsed + validated internally)
    cfg.pipeline = fastdem::parseConfig(yaml);
    cfg.postprocess = fastdem::config::parsePostProcess(yaml);

    // Validate node-level config
    cfg.validate();

    return cfg;
  }

 private:
  void validate() const {
    if (topics.input_scan.empty())
      throw std::invalid_argument("input_scan topic is empty");
    if (map.width <= 0.0 || map.height <= 0.0 || map.resolution <= 0.0)
      throw std::invalid_argument(
          "Invalid map geometry (all must be > 0): width=" +
          std::to_string(map.width) + ", height=" + std::to_string(map.height) +
          ", resolution=" + std::to_string(map.resolution));
    if (topics.publish_rate <= 0.0)
      throw std::invalid_argument("Invalid publish_rate: " +
                                  std::to_string(topics.publish_rate));
    if (tf.max_wait_time < 0.0)
      throw std::invalid_argument("Invalid max_wait_time: " +
                                  std::to_string(tf.max_wait_time));
    if (tf.max_stale_time < 0.0)
      throw std::invalid_argument("Invalid max_stale_time: " +
                                  std::to_string(tf.max_stale_time));
  }
};

}  // namespace fastdem::ros2

#endif  // FASTDEM_ROS_PARAMETERS_HPP
