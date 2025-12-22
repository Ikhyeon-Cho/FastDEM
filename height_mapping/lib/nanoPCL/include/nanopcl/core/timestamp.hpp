// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_TIMESTAMP_HPP
#define NANOPCL_CORE_TIMESTAMP_HPP

#include <cstdint>

namespace nanopcl {

/**
 * @brief Timestamp type for temporal metadata (nanoseconds since epoch)
 *
 * Follows ROS convention for temporal data representation.
 * Used by PointCloud and other time-aware geometry types.
 *
 * @note Stored as uint64_t nanoseconds since epoch (ROS convention)
 * @note Compatible with rclcpp::Time and builtin_interfaces::msg::Time
 */
using Timestamp = uint64_t;

}  // namespace nanopcl

#endif  // NANOPCL_CORE_TIMESTAMP_HPP
