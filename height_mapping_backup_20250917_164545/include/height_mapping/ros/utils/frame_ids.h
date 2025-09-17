#pragma once

#include <string>
#include <ros/node_handle.h>

namespace frame_ids {

inline std::string ROBOT_BASE;
inline std::string MAP;

namespace sensor {
inline std::string LIDAR;
inline std::string RGBD;
} // namespace sensor

// Initialize all frame IDs from ROS parameters
inline void loadFromConfig(const ros::NodeHandle &nh) {

  nh.param<std::string>("robot", ROBOT_BASE, "base_link");
  nh.param<std::string>("map", MAP, "map");
  nh.param<std::string>("lidar_sensor", sensor::LIDAR, "");
  nh.param<std::string>("rgbd_sensor", sensor::RGBD, "");
}
} // namespace frame_ids