#pragma once

#include <ros/node_handle.h>
#include "height_mapping/algorithms/HeightMapper.h"
#include "height_mapping/algorithms/GlobalMapper.h"

namespace height_mapping_ros {

namespace height_mapper {
static height_mapping::HeightMapper::Config loadConfig(const ros::NodeHandle &nh) {

  height_mapping::HeightMapper::Config cfg;
  nh.param<std::string>("estimator_type", cfg.estimator_type, "StatMean");
  nh.param<std::string>("frame_id", cfg.frame_id, "map");
  nh.param<double>("map_length_x", cfg.map_length_x, 10.0);
  nh.param<double>("map_length_y", cfg.map_length_y, 10.0);
  nh.param<double>("grid_resolution", cfg.grid_resolution, 0.1);
  nh.param<double>("min_height_threshold", cfg.min_height, -0.2);
  nh.param<double>("max_height_threshold", cfg.max_height, 1.5);
  return cfg;
}

} // namespace height_mapper

namespace global_mapper {
static height_mapping::GlobalMapper::Config loadConfig(const ros::NodeHandle &nh) {

  height_mapping::GlobalMapper::Config cfg;
  nh.param<std::string>("estimator_type", cfg.estimator_type, "StatMean");
  nh.param<std::string>("frame_id", cfg.frame_id, "map");
  nh.param<double>("map_length_x", cfg.map_length_x, 200.0);
  nh.param<double>("map_length_y", cfg.map_length_y, 200.0);
  nh.param<double>("grid_resolution", cfg.grid_resolution, 0.1);
  nh.param<double>("min_height_threshold", cfg.min_height, -0.2);
  nh.param<double>("max_height_threshold", cfg.max_height, 1.5);
  nh.param<std::string>("map_save_dir",
                        cfg.map_save_dir,
                        std::string("/home/") + std::getenv("USER") + "/Downloads");
  return cfg;
}

} // namespace global_mapper

} // namespace height_mapping_ros