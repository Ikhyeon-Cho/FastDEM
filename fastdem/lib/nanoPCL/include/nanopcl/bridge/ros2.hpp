// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Bridge module: ROS2 sensor_msgs/msg/PointCloud2 conversion.
//
// Usage:
//   #include <nanopcl/bridge/ros2.hpp>
//
//   // PointCloud2 -> nanoPCL
//   auto cloud = nanopcl::from(msg);
//
//   // nanoPCL -> PointCloud2 (using cloud's metadata)
//   auto out_msg = nanopcl::to(cloud);
//
//   // nanoPCL -> PointCloud2 (explicit metadata)
//   auto out_msg = nanopcl::to(cloud, "map", now());

#ifndef NANOPCL_BRIDGE_ROS2_HPP
#define NANOPCL_BRIDGE_ROS2_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "nanopcl/bridge/ros/impl.hpp"

namespace nanopcl {

namespace detail {

/// @brief Convert nanoseconds to ROS2 Time
inline builtin_interfaces::msg::Time toRos2Time(uint64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000ULL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);
  return t;
}

/// @brief Convert ROS2 Time to nanoseconds
inline uint64_t fromRos2Time(const builtin_interfaces::msg::Time& t) {
  return static_cast<uint64_t>(t.sec) * 1000000000ULL +
         static_cast<uint64_t>(t.nanosec);
}

} // namespace detail

/**
 * @brief Convert ROS2 PointCloud2 message to nanoPCL PointCloud
 *
 * Automatically detects and converts available fields:
 * - x, y, z (required)
 * - intensity (UINT8, UINT16, FLOAT32 -> float)
 * - ring (UINT8, UINT16 -> uint16_t)
 * - t/time/timestamp (UINT32, FLOAT32, FLOAT64 -> float)
 * - rgb/rgba (packed uint32 -> Color)
 * - label (UINT8, UINT16, UINT32 -> Label)
 * - normal_x, normal_y, normal_z (float -> Normal4)
 *
 * @param msg ROS2 PointCloud2 message
 * @return PointCloud with detected channels enabled
 *
 * @note Invalid points (NaN/Inf) are filtered out.
 * @note frame_id and timestamp are stored in the returned cloud.
 */
[[nodiscard]] inline PointCloud from(
    const sensor_msgs::msg::PointCloud2& msg) {
  auto cloud = detail::from_impl<sensor_msgs::msg::PointCloud2,
                                 sensor_msgs::msg::PointField>(msg);
  cloud.setTimestamp(detail::fromRos2Time(msg.header.stamp));
  return cloud;
}

/**
 * @brief Convert nanoPCL PointCloud to ROS2 PointCloud2 message
 *
 * Uses the cloud's internal frame_id and timestamp.
 *
 * @param cloud nanoPCL point cloud
 * @return ROS2 PointCloud2 message
 *
 * @note Only active channels are included in the output message.
 */
[[nodiscard]] inline sensor_msgs::msg::PointCloud2 to(const PointCloud& cloud) {
  return detail::to_impl<sensor_msgs::msg::PointCloud2,
                         sensor_msgs::msg::PointField>(
      cloud, cloud.frameId(), detail::toRos2Time(cloud.timestamp()));
}

/**
 * @brief Convert nanoPCL PointCloud to ROS2 PointCloud2 message with explicit
 * metadata
 *
 * @param cloud nanoPCL point cloud
 * @param frame_id Frame ID for message header
 * @param stamp Timestamp for message header
 * @return ROS2 PointCloud2 message
 *
 * @note Only active channels are included in the output message.
 */
[[nodiscard]] inline sensor_msgs::msg::PointCloud2 to(
    const PointCloud& cloud, const std::string& frame_id, const builtin_interfaces::msg::Time& stamp) {
  return detail::to_impl<sensor_msgs::msg::PointCloud2,
                         sensor_msgs::msg::PointField>(cloud, frame_id, stamp);
}

} // namespace nanopcl

#endif // NANOPCL_BRIDGE_ROS2_HPP
