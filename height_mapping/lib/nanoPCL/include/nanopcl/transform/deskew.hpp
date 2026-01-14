// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Motion compensation (deskewing) for spinning LiDAR sensors.

#ifndef NANOPCL_TRANSFORM_DESKEW_HPP
#define NANOPCL_TRANSFORM_DESKEW_HPP

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/transform/math.hpp"

namespace npcl {

/**
 * @brief Apply motion compensation (deskewing) to a point cloud.
 *
 * Corrects distortion caused by sensor motion during the scan.
 * Transforms all points to the coordinate frame of the sensor at the end of the
 * scan.
 *
 * @param cloud [in/out] Point cloud to deskew. Must have a 'time' channel.
 *                       'time' values should be normalized [0, 1] relative to
 * scan.
 * @param T_start Sensor pose at the start of the scan (relative to world/map).
 * @param T_end Sensor pose at the end of the scan (relative to world/map).
 *
 * @note This implementation uses a Constant Velocity Model (Linear
 * Interpolation).
 */
inline void deskew(PointCloud& cloud, const Eigen::Isometry3d& T_start,
                   const Eigen::Isometry3d& T_end) {
  if (!cloud.hasTime()) return;
  if (cloud.empty()) return;

  const auto& times = cloud.time();

  // Transformation from sensor frame at start to sensor frame at end
  // T_end_start = T_world_end^-1 * T_world_start
  Eigen::Isometry3d T_rel = T_end.inverse() * T_start;

  // T_rel represents the motion from End back to Start.
  // At t=0 (start), we apply T_rel.
  // At t=1 (end), we apply Identity.

#if defined(_OPENMP)
#pragma omp parallel for schedule(dynamic)
#endif
  for (size_t i = 0; i < cloud.size(); ++i) {
    float ratio = times[i];

    // Clamp ratio to [0, 1] to be safe
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    // Interpolate transformation from T_rel (start) to Identity (end)
    Eigen::Isometry3d T_corr = math::slerp(T_rel, Eigen::Isometry3d::Identity(),
                                           static_cast<double>(ratio));

    // Apply correction: cast to float and transform the point
    Eigen::Matrix3f R = T_corr.rotation().cast<float>();
    Eigen::Vector3f t = T_corr.translation().cast<float>();
    cloud.xyz()[i] = R * cloud.xyz()[i] + t;
  }
}

}  // namespace npcl

#endif  // NANOPCL_TRANSFORM_DESKEW_HPP
