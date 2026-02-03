// nanoPCL - Point cloud rigid body transformation
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_TRANSFORM_HPP
#define NANOPCL_CORE_TRANSFORM_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {

// Points and Normals are transformed. Covariances are NOT.
// For double precision poses, use T.cast<float>().

namespace detail {

inline void transformInPlace(PointCloud& cloud, const Eigen::Matrix4f& T,
                             const std::string& target_frame = "") {
  if (cloud.empty())
    return;

  auto& points = cloud.points();
  const size_t n = points.size();
  for (size_t i = 0; i < n; ++i)
    points[i] = T * points[i];

  if (cloud.hasNormal()) {
    auto& normals = cloud.normals();
    for (size_t i = 0; i < n; ++i)
      normals[i] = T * normals[i];
  }

  if (!target_frame.empty())
    cloud.setFrameId(target_frame);
}

} // namespace detail

// Copy: auto result = transformCloud(cloud, T);
[[nodiscard]] inline PointCloud transformCloud(const PointCloud& cloud,
                                               const Eigen::Matrix4f& T,
                                               const std::string& target_frame = "") {
  PointCloud result = cloud;
  detail::transformInPlace(result, T, target_frame);
  return result;
}

// In-place: cloud = transformCloud(std::move(cloud), T);
[[nodiscard]] inline PointCloud transformCloud(PointCloud&& cloud,
                                               const Eigen::Matrix4f& T,
                                               const std::string& target_frame = "") {
  detail::transformInPlace(cloud, T, target_frame);
  return std::move(cloud);
}

// Isometry3f overloads
[[nodiscard]] inline PointCloud transformCloud(const PointCloud& cloud,
                                               const Eigen::Isometry3f& T,
                                               const std::string& target_frame = "") {
  return transformCloud(cloud, T.matrix(), target_frame);
}

[[nodiscard]] inline PointCloud transformCloud(PointCloud&& cloud,
                                               const Eigen::Isometry3f& T,
                                               const std::string& target_frame = "") {
  return transformCloud(std::move(cloud), T.matrix(), target_frame);
}

// Isometry3d overloads
[[nodiscard]] inline PointCloud transformCloud(const PointCloud& cloud,
                                               const Eigen::Isometry3d& T,
                                               const std::string& target_frame = "") {
  return transformCloud(cloud, T.matrix().cast<float>(), target_frame);
}

[[nodiscard]] inline PointCloud transformCloud(PointCloud&& cloud,
                                               const Eigen::Isometry3d& T,
                                               const std::string& target_frame = "") {
  return transformCloud(std::move(cloud), T.matrix().cast<float>(), target_frame);
}

} // namespace nanopcl

#endif // NANOPCL_CORE_TRANSFORM_HPP
