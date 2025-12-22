/*
 * node_parameters.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ROS_NODE_PARAMETERS_H
#define HEIGHT_MAPPING_ROS_NODE_PARAMETERS_H

#include <ros/ros.h>

#include <string>

#include "height_mapping_ros/adapters/tf2.h"

namespace height_mapping_ros {

struct NodeParameters {
  std::string config_path{""};  // Mapper config file path
  std::string logger_level{"info"};
  bool use_configurable_mapper{false};  // false: HeightMapper

  struct Topics {
    std::string input_scan{"/points"};
    double publish_rate{10.0};
  } topics;

  adapters::TF2::Config tf;

  /// Load parameters from ROS parameter server
  static NodeParameters load(::ros::NodeHandle& nh) {
    NodeParameters params;

    auto load_param = [&nh](const std::string& name, auto& value) {
      value = nh.param(name, value);
    };

    load_param("config_path", params.config_path);
    load_param("use_configurable_mapper", params.use_configurable_mapper);
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
    if (config_path.empty()) {
      ROS_ERROR("config_path is empty. Specify a valid YAML config file path.");
      return false;
    }

    if (topics.publish_rate <= 0.0) {
      ROS_ERROR("Invalid publish_rate: %.2f (must be > 0)",
                topics.publish_rate);
      return false;
    }

    if (tf.max_wait_time < 0.0) {
      ROS_ERROR("Invalid max_wait_time: %.2f (must be >= 0)", tf.max_wait_time);
      return false;
    }

    if (tf.max_stale_time < 0.0) {
      ROS_ERROR("Invalid max_stale_time: %.2f (must be >= 0)",
                tf.max_stale_time);
      return false;
    }

    return true;
  }
};

}  // namespace height_mapping_ros

#endif  // HEIGHT_MAPPING_ROS_NODE_PARAMETERS_H
