/*
 * params.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ROS_PARAMS_H
#define HEIGHT_MAPPING_ROS_PARAMS_H

#include <ros/ros.h>
#include <string>

namespace height_mapping::ros {

struct Parameters {
  std::string mapping_config{""};

  // Topics
  std::string input_cloud_topic{"/points"};
  std::string output_map_topic{"height_map"};
  std::string processed_cloud_topic{"processed_cloud"};

  // Publishing settings
  double publish_rate_hz{10.0};
  bool publish_processed_cloud{true};

  // Transform
  double tf_timeout_s{0.1};
  double tf_extrapolation_s{0.05};

  // Default constructor
  Parameters() = default;

  // Constructor that loads from ROS parameter server
  explicit Parameters(::ros::NodeHandle &nh) {
    // Helper lambda for cleaner parameter loading
    auto load = [&nh](const std::string &name, auto &value) {
      value = nh.param(name, value);
    };

    // Mapping configuration
    load("mapping_config", mapping_config);

    // Topics
    load("input_cloud_topic", input_cloud_topic);
    load("output_map_topic", output_map_topic);
    load("processed_cloud_topic", processed_cloud_topic);

    // Publishing settings
    load("publish_rate", publish_rate_hz);
    load("publish_processed_cloud", publish_processed_cloud);

    // Transform settings
    load("tf_timeout", tf_timeout_s);
    load("tf_extrapolation", tf_extrapolation_s);
  }
};

} // namespace height_mapping::ros

#endif // HEIGHT_MAPPING_ROS_PARAMS_H