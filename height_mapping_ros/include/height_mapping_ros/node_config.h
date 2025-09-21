/*
 * node_config.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ROS_NODE_CONFIG_H
#define HEIGHT_MAPPING_ROS_NODE_CONFIG_H

#include <ros/ros.h>
#include <string>

namespace height_mapping::ros {

struct MappingNodeConfig {

  struct Topics {
    std::string point_cloud = "/points";
    std::string height_map = "height_map";
    std::string processed_cloud = "processed_cloud";
  } topics;

  struct Publishing {
    double rate = 10.0; // Hz
    bool publish_processed_cloud = false;
    bool publish_markers = true;
  } publishing;

  struct Processing {
    int queue_size = 10;
  } processing;

  struct Transform {
    double lookup_timeout = 0.1;      // seconds
    double extrapolation_time = 0.05; // seconds
  } transform;

  // Path to core configuration file
  std::string core_config_file = "";

  // Load from ROS parameter server
  static MappingNodeConfig loadFromROSParameters(::ros::NodeHandle &pnh) {
    MappingNodeConfig config;

    // Load ROS topics
    config.topics.point_cloud =
        pnh.param<std::string>("topics/point_cloud", config.topics.point_cloud);
    config.topics.height_map =
        pnh.param<std::string>("topics/height_map", config.topics.height_map);
    config.topics.processed_cloud = pnh.param<std::string>(
        "topics/processed_cloud", config.topics.processed_cloud);

    // Load publishing settings
    config.publishing.rate =
        pnh.param<double>("publishing/rate", config.publishing.rate);
    config.publishing.publish_processed_cloud =
        pnh.param<bool>("publishing/processed_cloud",
                        config.publishing.publish_processed_cloud);
    config.publishing.publish_markers = pnh.param<bool>(
        "publishing/markers", config.publishing.publish_markers);

    // Load processing settings
    config.processing.queue_size =
        pnh.param<int>("processing/queue_size", config.processing.queue_size);

    // Load TF settings
    config.transform.lookup_timeout = pnh.param<double>(
        "transform/lookup_timeout", config.transform.lookup_timeout);
    config.transform.extrapolation_time = pnh.param<double>(
        "transform/extrapolation_time", config.transform.extrapolation_time);

    // Path to core configuration file
    config.core_config_file = pnh.param<std::string>("core_config_file", "");

    return config;
  }

  // Validate configuration
  bool validate() const {
    // Check if core config file is specified
    if (core_config_file.empty()) {
      ROS_ERROR("Core configuration file not specified. Set 'core_config_file' "
                "parameter.");
      return false;
    }

    // Validate queue size
    if (processing.queue_size <= 0) {
      ROS_ERROR("Invalid queue size: %d", processing.queue_size);
      return false;
    }

    // Validate TF timeouts
    if (transform.lookup_timeout <= 0) {
      ROS_ERROR("Invalid TF lookup timeout: %.2f", transform.lookup_timeout);
      return false;
    }

    return true;
  }
};

} // namespace height_mapping::ros

#endif // HEIGHT_MAPPING_ROS_NODE_CONFIG_H