// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_ROS_PARAMETERS_HPP
#define FASTDEM_ROS_PARAMETERS_HPP

#include <ros/node_handle.h>
#include <spdlog/spdlog.h>

#include <string>

namespace fastdem::ros1 {

struct NodeParameters {
  std::string mapping_config{""};
  std::string mapping_mode{""};  // Override mapping mode ("local"/"global")
  std::string logger_level{"info"};

  struct Map {
    double width{15.0};
    double height{15.0};
    double resolution{0.1};
  } map;

  struct Topics {
    std::string input_scan{"/points"};
    double publish_rate{10.0};
  } topics;

  struct TF {
    std::string base_frame{"base_link"};
    std::string map_frame{"map"};
    double max_wait_time = 0.1;
    double max_stale_time = 0.1;
  } tf;

  /// Load parameters from ROS parameter server
  static NodeParameters load(ros::NodeHandle& nh) {
    NodeParameters params;

    auto load_param = [&nh](const std::string& name, auto& value) {
      value = nh.param(name, value);
    };

    load_param("mapping_config", params.mapping_config);
    load_param("map/width", params.map.width);
    load_param("map/height", params.map.height);
    load_param("map/resolution", params.map.resolution);
    load_param("mapping_mode", params.mapping_mode);
    load_param("topics/input_scan", params.topics.input_scan);
    load_param("topics/publish_rate", params.topics.publish_rate);
    load_param("tf/base_frame", params.tf.base_frame);
    load_param("tf/map_frame", params.tf.map_frame);
    load_param("tf/max_wait_time", params.tf.max_wait_time);
    load_param("tf/max_stale_time", params.tf.max_stale_time);
    load_param("logger/level", params.logger_level);

    return params;
  }

  bool isValid() const {
    if (mapping_config.empty()) {
      spdlog::error(
          "mapping_config is empty. Specify a valid YAML config file path.");
      return false;
    }

    if (map.width <= 0.0 || map.height <= 0.0 || map.resolution <= 0.0) {
      spdlog::error(
          "Invalid map geometry: width={}, height={}, resolution={} (all must "
          "be > 0)",
          map.width, map.height, map.resolution);
      return false;
    }

    if (topics.publish_rate <= 0.0) {
      spdlog::error("Invalid publish_rate: {} (must be > 0)",
                    topics.publish_rate);
      return false;
    }

    if (tf.max_wait_time < 0.0) {
      spdlog::error("Invalid max_wait_time: {} (must be >= 0)",
                    tf.max_wait_time);
      return false;
    }

    if (tf.max_stale_time < 0.0) {
      spdlog::error("Invalid max_stale_time: {} (must be >= 0)",
                    tf.max_stale_time);
      return false;
    }

    return true;
  }
};

}  // namespace fastdem::ros1

#endif  // FASTDEM_ROS_PARAMETERS_HPP
