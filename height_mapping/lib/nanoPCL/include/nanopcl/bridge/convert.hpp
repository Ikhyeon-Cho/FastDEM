// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Bridge module: Type conversion traits for external formats.
//
// This header defines the Convert<T> traits template that enables
// seamless conversion between nanoPCL and external message formats.
// Include specific bridge headers (ros2.hpp, pcl.hpp) for specializations.

#ifndef NANOPCL_BRIDGE_CONVERT_HPP
#define NANOPCL_BRIDGE_CONVERT_HPP

#include "nanopcl/core/point_cloud.hpp"

namespace npcl {

/**
 * @brief Type conversion traits for external point cloud formats
 *
 * Specialize this template for your message type to enable automatic
 * conversion via from<T>() and to<T>() functions.
 *
 * Required static methods in specialization:
 *   - static PointCloud from(const T& msg)
 *   - static T to(const PointCloud& cloud)
 *
 * @tparam T External message type (e.g., sensor_msgs::msg::PointCloud2)
 *
 * Example specialization:
 * @code
 * template<>
 * struct Convert<MyCustomMsg> {
 *     static PointCloud from(const MyCustomMsg& msg) { ... }
 *     static MyCustomMsg to(const PointCloud& cloud) { ... }
 * };
 * @endcode
 */
template <typename T, typename = void>
struct Convert {
  // Primary template is intentionally undefined.
  // Attempting to use an unsupported type will produce a clear compile error.

  static_assert(sizeof(T) == 0,
                "Convert<T> is not specialized for this type. "
                "Include the appropriate bridge header (e.g., "
                "nanopcl/bridge/ros2.hpp) "
                "or provide your own specialization.");
};

// =============================================================================
// Convenience Functions
// =============================================================================

/**
 * @brief Convert external message to PointCloud
 *
 * @tparam T External message type (auto-deduced)
 * @param msg External message to convert
 * @return PointCloud converted from the message
 *
 * @code
 * #include <nanopcl/bridge/ros2.hpp>
 * auto cloud = npcl::from(ros_msg);
 * @endcode
 */
template <typename T>
[[nodiscard]] inline PointCloud from(const T& msg) {
  return Convert<T>::from(msg);
}

/**
 * @brief Convert PointCloud to external message
 *
 * @tparam T Target external message type (must be specified)
 * @param cloud PointCloud to convert
 * @return T External message converted from the cloud
 *
 * @code
 * #include <nanopcl/bridge/ros2.hpp>
 * auto msg = npcl::to<sensor_msgs::msg::PointCloud2>(cloud);
 * @endcode
 */
template <typename T>
[[nodiscard]] inline T to(const PointCloud& cloud) {
  return Convert<T>::to(cloud);
}

/**
 * @brief Convert PointCloud to external message (shared_ptr overload)
 *
 * @tparam T Target external message type (must be specified)
 * @param cloud Shared pointer to PointCloud
 * @return T External message converted from the cloud
 */
template <typename T>
[[nodiscard]] inline T to(const std::shared_ptr<PointCloud>& cloud) {
  return Convert<T>::to(*cloud);
}

/**
 * @brief Convert PointCloud to external message (shared_ptr const overload)
 */
template <typename T>
[[nodiscard]] inline T to(const std::shared_ptr<const PointCloud>& cloud) {
  return Convert<T>::to(*cloud);
}

}  // namespace npcl

#endif  // NANOPCL_BRIDGE_CONVERT_HPP
