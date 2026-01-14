// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Bridge module: ROS 1 sensor_msgs::PointCloud2 conversion.
//
// Usage:
//   #include <nanopcl/bridge/ros1.hpp>
//
//   void callback(const sensor_msgs::PointCloud2& msg) {
//       auto cloud = npcl::from(msg);
//       // ... process ...
//       auto out = npcl::to<sensor_msgs::PointCloud2>(cloud);
//   }

#ifndef NANOPCL_BRIDGE_ROS1_HPP
#define NANOPCL_BRIDGE_ROS1_HPP

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "nanopcl/bridge/convert.hpp"
#include "nanopcl/transform/transform.hpp"

namespace npcl {

// =============================================================================
// PointCloud2 Conversion (via Convert<> specialization)
// =============================================================================

/**
 * @brief Convert ROS 1 PointCloud2 message to nanoPCL PointCloud
 *
 * This function performs a **Deep Copy** of the point data from the message
 * into the SoA-aligned nanoPCL container. It automatically detects and maps
 * fields like `intensity`, `ring`, `time`, `rgb`, and `label`.
 *
 * @param msg ROS 1 PointCloud2 message
 * @return PointCloud populated with message data
 *
 * @note **Performance**: For 100k points, conversion typically takes ~1-2ms.
 */
template <>
struct Convert<sensor_msgs::PointCloud2>;

/**
 * @brief Validate ROS 1 PointCloud2 message
 *
 * @param msg PointCloud2 message to validate
 * @return true if message has valid frame_id and XYZ fields
 */
[[nodiscard]] bool validate(const sensor_msgs::PointCloud2& msg);

// =============================================================================
// Transform Conversion
// =============================================================================

/**
 * @brief Convert ROS 1 Transform to nanoPCL Transform (frameless)
 *
 * @param msg geometry_msgs::Transform message
 * @param parent Parent frame name (target frame)
 * @param child Child frame name (source frame)
 * @return Transform converted transform
 */
[[nodiscard]] Transform from(const geometry_msgs::Transform& msg,
                             const std::string& parent,
                             const std::string& child);

/**
 * @brief Convert ROS 1 TransformStamped to nanoPCL Transform
 *
 * @param msg geometry_msgs::TransformStamped message
 * @return Transform converted transform with frame information
 *
 * @code
 * geometry_msgs::TransformStamped tf_msg;
 * tf_buffer.lookupTransform("map", "base_link", ros::Time(0), tf_msg);
 * auto tf = npcl::from(tf_msg);
 * @endcode
 */
[[nodiscard]] Transform from(const geometry_msgs::TransformStamped& msg);

/**
 * @brief Convert nanoPCL Transform to ROS 1 Transform
 *
 * @param tf nanoPCL Transform
 * @return geometry_msgs::Transform converted message
 */
[[nodiscard]] geometry_msgs::Transform toTransformMsg(const Transform& tf);

/**
 * @brief Convert nanoPCL Transform to ROS 1 TransformStamped
 *
 * @param tf nanoPCL Transform
 * @param stamp ROS timestamp (default: ros::Time(0))
 * @return geometry_msgs::TransformStamped converted message
 *
 * @code
 * auto tf = Transform::from2D("map", "base_link", x, y, yaw);
 * auto msg = npcl::toTransformStampedMsg(tf, ros::Time::now());
 * tf_broadcaster.sendTransform(msg);
 * @endcode
 */
[[nodiscard]] geometry_msgs::TransformStamped toTransformStampedMsg(
    const Transform& tf, const ros::Time& stamp = ros::Time(0));

}  // namespace npcl

#include "nanopcl/bridge/impl/ros1_impl.hpp"

#endif  // NANOPCL_BRIDGE_ROS1_HPP
