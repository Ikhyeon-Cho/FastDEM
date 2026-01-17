// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Bridge module: ROS 2 sensor_msgs::msg::PointCloud2 conversion.
//
// Usage:
//   #include <nanopcl/bridge/ros2.hpp>
//
//   void callback(const sensor_msgs::msg::PointCloud2& msg) {
//       auto cloud = npcl::from(msg);
//       // ... process ...
//       auto out = npcl::to<sensor_msgs::msg::PointCloud2>(cloud);
//   }

#ifndef NANOPCL_BRIDGE_ROS2_HPP
#define NANOPCL_BRIDGE_ROS2_HPP

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "nanopcl/bridge/convert.hpp"
#include "nanopcl/transform/transform.hpp"

namespace npcl {

// =============================================================================
// PointCloud2 Conversion (via Convert<> specialization)
// =============================================================================

/**
 * @brief Convert specialization for ROS 2 PointCloud2
 *
 * Enables:
 *   auto cloud = npcl::from(msg);
 *   auto msg = npcl::to<sensor_msgs::msg::PointCloud2>(cloud);
 */
template <>
struct Convert<sensor_msgs::msg::PointCloud2>;

/**
 * @brief Validate ROS 2 PointCloud2 message
 *
 * @param msg PointCloud2 message to validate
 * @return true if message has valid frame_id and XYZ fields
 */
[[nodiscard]] bool validate(const sensor_msgs::msg::PointCloud2& msg);

// =============================================================================
// Transform Conversion
// =============================================================================

/**
 * @brief Convert ROS 2 Transform to nanoPCL Transform (frameless)
 *
 * @param msg geometry_msgs::msg::Transform message
 * @param parent Parent frame name (target frame)
 * @param child Child frame name (source frame)
 * @return Transform converted transform
 */
[[nodiscard]] Transform from(const geometry_msgs::msg::Transform& msg,
                             const std::string& parent,
                             const std::string& child);

/**
 * @brief Convert ROS 2 TransformStamped to nanoPCL Transform
 *
 * @param msg geometry_msgs::msg::TransformStamped message
 * @return Transform converted transform with frame information
 *
 * @code
 * auto tf_msg = tf_buffer->lookupTransform("map", "base_link",
 *                                          tf2::TimePointZero);
 * auto tf = npcl::from(tf_msg);
 * @endcode
 */
[[nodiscard]] Transform from(const geometry_msgs::msg::TransformStamped& msg);

/**
 * @brief Convert nanoPCL Transform to ROS 2 Transform
 *
 * @param tf nanoPCL Transform
 * @return geometry_msgs::msg::Transform converted message
 */
[[nodiscard]] geometry_msgs::msg::Transform toTransformMsg(const Transform& tf);

/**
 * @brief Convert nanoPCL Transform to ROS 2 TransformStamped
 *
 * @param tf nanoPCL Transform
 * @param stamp ROS 2 timestamp (default: zero time)
 * @return geometry_msgs::msg::TransformStamped converted message
 *
 * @code
 * auto tf = Transform::from2D("map", "base_link", x, y, yaw);
 * auto msg = npcl::toTransformStampedMsg(tf, node->now());
 * tf_broadcaster->sendTransform(msg);
 * @endcode
 */
[[nodiscard]] geometry_msgs::msg::TransformStamped toTransformStampedMsg(
    const Transform& tf,
    const builtin_interfaces::msg::Time& stamp = builtin_interfaces::msg::Time());

}  // namespace npcl

#include "nanopcl/bridge/impl/ros2_impl.hpp"

#endif  // NANOPCL_BRIDGE_ROS2_HPP
