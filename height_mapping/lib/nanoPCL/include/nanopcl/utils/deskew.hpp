// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_UTILS_DESKEW_HPP
#define NANOPCL_UTILS_DESKEW_HPP

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/transform/se3.hpp"

namespace nanopcl {
namespace utils {

/**
 * @brief Apply motion compensation (deskewing) to a point cloud.
 * 
 * Corrects distortion caused by sensor motion during the scan.
 * Transforms all points to the coordinate frame of the sensor at the end of the scan.
 * 
 * @param cloud [in/out] Point cloud to deskew. Must have a 'time' channel.
 *                       'time' values should be normalized [0, 1] relative to scan.
 * @param T_start Sensor pose at the start of the scan (relative to world/map).
 * @param T_end Sensor pose at the end of the scan (relative to world/map).
 * 
 * @note This implementation uses a Constant Velocity Model (Linear Interpolation).
 */
inline void deskew(PointCloud& cloud, const SE3d& T_start, const SE3d& T_end) {
  if (!cloud.hasTime()) return;
  if (cloud.empty()) return;

  const auto& times = cloud.time();
  
  // Transformation from sensor frame at start to sensor frame at end
  // T_end_start = T_world_end^-1 * T_world_start
  SE3d T_rel = T_end.inverse() * T_start;
  
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
    SE3d T_corr = T_rel.slerp(SE3d::Identity(), static_cast<double>(ratio));
    
    // Apply correction
    cloud[i] = T_corr.cast<float>() * cloud[i];
  }
}

}  // namespace utils
}  // namespace nanopcl

#endif  // NANOPCL_UTILS_DESKEW_HPP
