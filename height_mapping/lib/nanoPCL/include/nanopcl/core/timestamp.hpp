// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_TIMESTAMP_HPP
#define NANOPCL_CORE_TIMESTAMP_HPP

#include <chrono>
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

// ============================================================================
// Time Utility Functions
// ============================================================================

namespace time {

/**
 * @brief Convert nanoseconds to seconds
 * @param ns Timestamp in nanoseconds
 * @return Time in seconds (double precision)
 *
 * @code
 * Timestamp stamp = cloud.timestamp();
 * double sec = time::toSec(stamp);
 * @endcode
 */
inline double toSec(Timestamp ns) { return static_cast<double>(ns) * 1e-9; }

/**
 * @brief Convert seconds to nanoseconds
 * @param sec Time in seconds
 * @return Timestamp in nanoseconds
 *
 * @code
 * double sec = 1703401845.123;
 * Timestamp stamp = time::fromSec(sec);
 * @endcode
 */
inline Timestamp fromSec(double sec) {
  return static_cast<Timestamp>(sec * 1e9);
}

/**
 * @brief Get current system time as nanoseconds since epoch
 * @return Current timestamp in nanoseconds
 *
 * @code
 * cloud.setTimestamp(time::now());
 * @endcode
 */
inline Timestamp now() {
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<Timestamp>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count());
}

/**
 * @brief Compute time difference between two timestamps
 * @param a First timestamp (nanoseconds)
 * @param b Second timestamp (nanoseconds)
 * @return Time difference in seconds (a - b)
 *
 * @note Uses integer subtraction before conversion to avoid precision loss
 *       from catastrophic cancellation when subtracting large similar values
 *
 * @code
 * double dt = time::diff(stamp_now, stamp_prev);  // seconds elapsed
 * @endcode
 */
inline double diff(Timestamp a, Timestamp b) {
  // Perform integer subtraction first to preserve precision
  if (a >= b) {
    return static_cast<double>(a - b) * 1e-9;
  } else {
    return -static_cast<double>(b - a) * 1e-9;
  }
}

}  // namespace time

}  // namespace nanopcl

#endif  // NANOPCL_CORE_TIMESTAMP_HPP
