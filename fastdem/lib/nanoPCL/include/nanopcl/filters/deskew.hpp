// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Deskew module: Motion distortion correction for spinning LiDAR.
//
// During a LiDAR scan (~100ms), the sensor moves, causing motion distortion.
// This module corrects the distortion by transforming each point to a common
// reference frame (T_end by default).
//
// Usage:
//   // Simple: Linear interpolation between start and end poses
//   auto corrected = filters::deskew(cloud, T_start, T_end);
//
//   // Advanced: Custom pose lookup (e.g., from trajectory or TF)
//   auto corrected = filters::deskew(cloud, [&](double t) {
//       return trajectory.poseAt(t);
//   });

#ifndef NANOPCL_FILTERS_DESKEW_HPP
#define NANOPCL_FILTERS_DESKEW_HPP

#include <Eigen/Geometry>
#include <functional>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace filters {

// =============================================================================
// Time Estimation Strategy
// =============================================================================

/// @brief Strategy for estimating point timestamps when time channel is missing
enum class TimeStrategy {
  CHANNEL, ///< Use existing time channel (auto-detect min/max range)
  INDEX    ///< Map point index 0..N to ratio 0.0..1.0
};

// =============================================================================
// Types
// =============================================================================

/// @brief Function type for pose lookup at a given timestamp
using PoseLookupFunc = std::function<Eigen::Isometry3d(double)>;

// =============================================================================
// Callback-based API (Advanced)
// =============================================================================

/// @brief Deskew using custom pose lookup function
///
/// @param cloud Input point cloud (must have time channel or use INDEX strategy)
/// @param pose_lookup Function that returns pose at given timestamp
/// @return Motion-corrected point cloud in the frame of the last point's pose
///
/// @note The pose_lookup function is called for each point's timestamp.
///       Ensure the function can handle the timestamp range in your data.
///
/// @code
/// // ROS TF2 example
/// auto corrected = filters::deskew(cloud, [&](double t) {
///     auto tf = tf_buffer.lookupTransform("odom", "base_link", ros::Time(t));
///     return nanopcl::from(tf);
/// });
///
/// // Trajectory example
/// auto corrected = filters::deskew(cloud, [&](double t) {
///     return trajectory.poseAt(t);
/// });
/// @endcode
[[nodiscard]] PointCloud deskew(const PointCloud& cloud,
                                PoseLookupFunc pose_lookup);

[[nodiscard]] PointCloud deskew(PointCloud&& cloud, PoseLookupFunc pose_lookup);

// =============================================================================
// Linear Interpolation API (Simple)
// =============================================================================

/// @brief Deskew using linear interpolation between start and end poses
///
/// @param cloud Input point cloud
/// @param T_start Pose at scan start (maps to min time or index 0)
/// @param T_end Pose at scan end (maps to max time or index N-1)
/// @param strategy Time estimation strategy (default: CHANNEL)
/// @return Motion-corrected point cloud in T_end frame
///
/// @note For CHANNEL strategy: automatically detects min/max time from data.
///       For INDEX strategy: assumes points are in temporal order (0..N -> 0..1).
///
/// @code
/// Eigen::Isometry3d T_start = odom.poseAt(scan_start_time);
/// Eigen::Isometry3d T_end = odom.poseAt(scan_end_time);
/// auto corrected = filters::deskew(cloud, T_start, T_end);
/// @endcode
[[nodiscard]] PointCloud deskew(const PointCloud& cloud,
                                const Eigen::Isometry3d& T_start,
                                const Eigen::Isometry3d& T_end,
                                TimeStrategy strategy = TimeStrategy::CHANNEL);

[[nodiscard]] PointCloud deskew(PointCloud&& cloud,
                                const Eigen::Isometry3d& T_start,
                                const Eigen::Isometry3d& T_end,
                                TimeStrategy strategy = TimeStrategy::CHANNEL);

/// @brief Deskew with explicit time range mapping
///
/// @param cloud Input point cloud (must have time channel)
/// @param T_start Pose at t_start
/// @param T_end Pose at t_end
/// @param t_start Timestamp corresponding to T_start
/// @param t_end Timestamp corresponding to T_end
/// @return Motion-corrected point cloud in T_end frame
///
/// @note Use this when your time channel contains absolute timestamps
///       (e.g., Unix epoch or GPS time) rather than normalized ratios.
///
/// @code
/// // Time channel contains values like 1678901234.567
/// auto corrected = filters::deskew(cloud, T_start, T_end, 1678901234.5, 1678901234.6);
/// @endcode
[[nodiscard]] PointCloud deskew(const PointCloud& cloud,
                                const Eigen::Isometry3d& T_start,
                                const Eigen::Isometry3d& T_end,
                                double t_start,
                                double t_end);

[[nodiscard]] PointCloud deskew(PointCloud&& cloud,
                                const Eigen::Isometry3d& T_start,
                                const Eigen::Isometry3d& T_end,
                                double t_start,
                                double t_end);

} // namespace filters
} // namespace nanopcl

#include "nanopcl/filters/impl/deskew_impl.hpp"

#endif // NANOPCL_FILTERS_DESKEW_HPP
