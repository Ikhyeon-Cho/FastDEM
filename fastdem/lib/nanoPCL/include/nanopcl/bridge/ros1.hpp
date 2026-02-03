// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Bridge module: ROS1 sensor_msgs/PointCloud2 conversion.
//
// Usage:
//   #include <nanopcl/bridge/ros1.hpp>
//
//   // PointCloud2 -> nanoPCL
//   auto cloud = nanopcl::from(msg);
//
//   // nanoPCL -> PointCloud2 (using cloud's metadata)
//   auto out_msg = nanopcl::to(cloud);
//
//   // nanoPCL -> PointCloud2 (explicit metadata)
//   auto out_msg = nanopcl::to(cloud, "map", ros::Time::now());

#ifndef NANOPCL_BRIDGE_ROS1_HPP
#define NANOPCL_BRIDGE_ROS1_HPP

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "nanopcl/bridge/ros/impl.hpp"

namespace nanopcl {

namespace detail {

/// @brief Convert nanoseconds to ROS1 Time
inline ros::Time toRos1Time(uint64_t ns) {
  return ros::Time(static_cast<uint32_t>(ns / 1000000000ULL),
                   static_cast<uint32_t>(ns % 1000000000ULL));
}

/// @brief Convert ROS1 Time to nanoseconds
inline uint64_t fromRos1Time(const ros::Time& t) {
  return static_cast<uint64_t>(t.sec) * 1000000000ULL +
         static_cast<uint64_t>(t.nsec);
}

} // namespace detail

/**
 * @brief Convert ROS1 PointCloud2 message to nanoPCL PointCloud
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
 * @param msg ROS1 PointCloud2 message
 * @return PointCloud with detected channels enabled
 *
 * @note Invalid points (NaN/Inf) are filtered out.
 * @note frame_id and timestamp are stored in the returned cloud.
 */
[[nodiscard]] inline PointCloud from(const sensor_msgs::PointCloud2& msg) {
  auto cloud =
      detail::from_impl<sensor_msgs::PointCloud2, sensor_msgs::PointField>(msg);
  cloud.setTimestamp(detail::fromRos1Time(msg.header.stamp));
  return cloud;
}

/**
 * @brief Convert nanoPCL PointCloud to ROS1 PointCloud2 message
 *
 * Uses the cloud's internal frame_id and timestamp.
 *
 * @param cloud nanoPCL point cloud
 * @return ROS1 PointCloud2 message
 *
 * @note Only active channels are included in the output message.
 */
[[nodiscard]] inline sensor_msgs::PointCloud2 to(const PointCloud& cloud) {
  return detail::to_impl<sensor_msgs::PointCloud2, sensor_msgs::PointField>(
      cloud, cloud.frameId(), detail::toRos1Time(cloud.timestamp()));
}

/**
 * @brief Convert nanoPCL PointCloud to ROS1 PointCloud2 message with explicit
 * metadata
 *
 * @param cloud nanoPCL point cloud
 * @param frame_id Frame ID for message header
 * @param stamp Timestamp for message header
 * @return ROS1 PointCloud2 message
 *
 * @note Only active channels are included in the output message.
 */
[[nodiscard]] inline sensor_msgs::PointCloud2 to(const PointCloud& cloud,
                                                 const std::string& frame_id,
                                                 const ros::Time& stamp) {
  return detail::to_impl<sensor_msgs::PointCloud2, sensor_msgs::PointField>(
      cloud, frame_id, stamp);
}

} // namespace nanopcl

#endif // NANOPCL_BRIDGE_ROS1_HPP
