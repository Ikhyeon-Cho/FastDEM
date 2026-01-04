// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

/**
 * @file nanopcl.hpp
 * @brief nanoPCL - Lightweight point cloud processing library
 *
 * A header-only C++17 library for point cloud processing.
 * No ROS. No PCL. Just Eigen.
 *
 * Features:
 * - Point cloud container with frame/timestamp metadata
 * - SE(3)/SO(3) rigid body transforms
 * - VoxelGrid and Passthrough filters
 * - Bounding boxes (AABB, OBB)
 * - Frame-aware transform operations
 *
 * Quick Start:
 * @code
 * #include <nanopcl/nanopcl.hpp>
 *
 * using namespace nanopcl;
 *
 * // Create point cloud
 * PointCloud cloud("lidar");
 * cloud.push_back(Point(1.0f, 2.0f, 3.0f));
 *
 * // Filter
 * cloud = filters::voxelGrid(cloud, 0.1f);
 *
 * // Transform
 * SE3d T = SE3d::from2D(1.0, 0.0, M_PI/4);
 * cloud = T * cloud;
 * @endcode
 */

#ifndef NANOPCL_HPP
#define NANOPCL_HPP

// Version information
#define NANOPCL_VERSION_MAJOR 0
#define NANOPCL_VERSION_MINOR 1
#define NANOPCL_VERSION_PATCH 0

// Core (includes: Point, PointCloud, Timestamp, Label, Color)
#include "nanopcl/core/point_cloud.hpp"

// Transform (includes: SO3, SE3, Transform)
#include "nanopcl/transform/transform.hpp"
#include "nanopcl/transform/provider.hpp"
#include "nanopcl/transform/transform_ops.hpp"

// Filters
#include "nanopcl/filters/filters.hpp"

// Geometry
#include "nanopcl/geometry/bbox.hpp"
#include "nanopcl/geometry/normal_estimation.hpp"

// Search
#include "nanopcl/search/voxel_hash_map.hpp"

// I/O
#include "nanopcl/io/pcd_io.hpp"
#include "nanopcl/io/bin_io.hpp"
#include "nanopcl/io/pose_io.hpp"

// Utils
#include "nanopcl/utils/deskew.hpp"

namespace nanopcl {

// Version constants
constexpr int VERSION_MAJOR = NANOPCL_VERSION_MAJOR;
constexpr int VERSION_MINOR = NANOPCL_VERSION_MINOR;
constexpr int VERSION_PATCH = NANOPCL_VERSION_PATCH;

/**
 * @brief Get version string
 */
inline std::string version() {
  return std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR) +
         "." + std::to_string(VERSION_PATCH);
}

}  // namespace nanopcl

#endif  // NANOPCL_HPP
