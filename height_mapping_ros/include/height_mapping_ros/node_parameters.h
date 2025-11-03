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

namespace height_mapping::ros {

struct NodeParameters {
  struct Topics {
    std::string scan{"/points"};
    std::string heightmap{"height_map"};
    std::string scan_processed{"scan_processed"};
  } topics;

  double publish_rate{10.0};

  struct TfTree {
    double lookup_timeout{0.1};
    double max_extrapolation_time{0.05};
  } tf_tree;

  struct Mapper {
    std::string config_file{""};
    struct Benchmark {
      bool enabled{false};
      int report_interval{100};
      bool log_each_stage{false};
    } benchmark;
  } mapper;

  std::string logger_level{"INFO"};  // DEBUG, INFO, WARN, ERROR

  NodeParameters(::ros::NodeHandle &nh) {
    // Helper lambda
    auto load = [&nh](const std::string &name, auto &value) {
      value = nh.param(name, value);
    };

    load("topics/scan", topics.scan);
    load("topics/scan_processed", topics.scan_processed);
    load("topics/heightmap", topics.heightmap);
    load("topics/publish_rate", publish_rate);

    // Transform settings
    load("tf_tree/lookup_timeout", tf_tree.lookup_timeout);
    load("tf_tree/max_extrapolation_time", tf_tree.max_extrapolation_time);

    // Mapper settings
    load("mapper/config_file", mapper.config_file);
    load("mapper/benchmark/enabled", mapper.benchmark.enabled);
    load("mapper/benchmark/report_interval", mapper.benchmark.report_interval);
    load("mapper/benchmark/log_each_stage", mapper.benchmark.log_each_stage);

    // Logger settings
    load("logger/level", logger_level);
  }
};

}  // namespace height_mapping::ros

#endif  // HEIGHT_MAPPING_ROS_NODE_PARAMETERS_H