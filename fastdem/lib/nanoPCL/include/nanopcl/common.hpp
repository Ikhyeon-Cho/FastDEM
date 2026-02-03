// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Common header: includes frequently used modules.
//
// For specialized functionality, include separately:
//   #include <nanopcl/io.hpp>          // File I/O (PCD, BIN, Pose)
//   #include <nanopcl/search.hpp>      // Spatial search (VoxelHash, KdTree)
//
// For external format conversion (requires external dependencies):
//   #include <nanopcl/bridge/ros1.hpp> // ROS 1 sensor_msgs (requires ROS 1)
//   #include <nanopcl/bridge/ros2.hpp> // ROS 2 sensor_msgs (requires ROS 2)
//   #include <nanopcl/bridge/pcl.hpp>  // PCL pcl::PointCloud (requires PCL)

#ifndef NANOPCL_COMMON_HPP
#define NANOPCL_COMMON_HPP

// Version information
#define NANOPCL_VERSION_MAJOR 0
#define NANOPCL_VERSION_MINOR 1
#define NANOPCL_VERSION_PATCH 0

// Frequently used modules
#include "nanopcl/core.hpp"
#include "nanopcl/filters.hpp"

namespace nanopcl {

// Version constants
constexpr int VERSION_MAJOR = NANOPCL_VERSION_MAJOR;
constexpr int VERSION_MINOR = NANOPCL_VERSION_MINOR;
constexpr int VERSION_PATCH = NANOPCL_VERSION_PATCH;

/// Get version string
inline std::string version() {
  return std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR) +
         "." + std::to_string(VERSION_PATCH);
}

} // namespace nanopcl

#endif // NANOPCL_COMMON_HPP
