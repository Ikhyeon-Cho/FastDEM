// nanopcl/transform/impl/transform_ops_impl.hpp
// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for transform operations.
// Do not include this file directly; include <nanopcl/transform/transform_ops.hpp>

#ifndef NANOPCL_TRANSFORM_IMPL_TRANSFORM_OPS_IMPL_HPP
#define NANOPCL_TRANSFORM_IMPL_TRANSFORM_OPS_IMPL_HPP

#include <stdexcept>

namespace nanopcl {
namespace detail {

// Internal helper: apply R*p + t to all points in-place
inline void applyTransformInPlace(PointCloud& cloud,
                                   const Eigen::Matrix3f& R,
                                   const Eigen::Vector3f& t) {
  for (auto& point : cloud.xyz()) {
    point = R * point + t;
  }
}

}  // namespace detail

// ============================================================================
// Frame-aware Transform API
// ============================================================================

// Copy version
template <typename Scalar>
PointCloud transform(const PointCloud& cloud, const Transform_<Scalar>& tf) {
  if (!tf.isValid()) {
    throw std::runtime_error("transform: Transform not initialized");
  }
  if (tf.child_frame() != cloud.frameId()) {
    throw std::runtime_error(
        "transform: frame mismatch: transform.from='" + tf.child_frame() +
        "' != cloud.frameId='" + cloud.frameId() + "'");
  }

  // Early return for empty clouds
  if (cloud.empty()) {
    PointCloud result;
    result.setFrameId(tf.parent_frame());
    result.setTimestamp(cloud.timestamp());
    return result;
  }

  // Deep copy (preserves all channels)
  PointCloud result = cloud;

  // Cast transform to float ONCE (optimal: "Points float, Poses double")
  const Eigen::Matrix3f R = tf.rotation().matrix().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  detail::applyTransformInPlace(result, R, t);
  result.setFrameId(tf.parent_frame());

  return result;
}

// Move version
template <typename Scalar>
PointCloud transform(PointCloud&& cloud, const Transform_<Scalar>& tf) {
  if (!tf.isValid()) {
    throw std::runtime_error("transform: Transform not initialized");
  }
  if (tf.child_frame() != cloud.frameId()) {
    throw std::runtime_error(
        "transform: frame mismatch: transform.from='" + tf.child_frame() +
        "' != cloud.frameId='" + cloud.frameId() + "'");
  }

  if (cloud.empty()) {
    cloud.setFrameId(tf.parent_frame());
    return std::move(cloud);
  }

  // Cast transform to float ONCE
  const Eigen::Matrix3f R = tf.rotation().matrix().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  detail::applyTransformInPlace(cloud, R, t);
  cloud.setFrameId(tf.parent_frame());

  return std::move(cloud);
}

// ============================================================================
// SE3 Transform API
// ============================================================================

// Copy version
template <typename Scalar>
PointCloud transform(const PointCloud& cloud, const SE3_<Scalar>& se3) {
  if (cloud.empty()) {
    PointCloud result;
    result.setFrameId(cloud.frameId());
    result.setTimestamp(cloud.timestamp());
    return result;
  }

  // Deep copy
  PointCloud result = cloud;

  // Cast SE3 to float ONCE
  const Eigen::Matrix3f R = se3.rotation().matrix().template cast<float>();
  const Eigen::Vector3f t = se3.translation().template cast<float>();

  detail::applyTransformInPlace(result, R, t);

  return result;
}

// Move version
template <typename Scalar>
PointCloud transform(PointCloud&& cloud, const SE3_<Scalar>& se3) {
  if (cloud.empty()) {
    return std::move(cloud);
  }

  // Cast SE3 to float ONCE
  const Eigen::Matrix3f R = se3.rotation().matrix().template cast<float>();
  const Eigen::Vector3f t = se3.translation().template cast<float>();

  detail::applyTransformInPlace(cloud, R, t);

  return std::move(cloud);
}

// ============================================================================
// Eigen::Transform API
// ============================================================================

// Copy version
template <typename Scalar, int Mode>
PointCloud transform(const PointCloud& cloud,
                     const Eigen::Transform<Scalar, 3, Mode>& tf) {
  if (cloud.empty()) {
    PointCloud result;
    result.setFrameId(cloud.frameId());
    result.setTimestamp(cloud.timestamp());
    return result;
  }

  // Deep copy
  PointCloud result = cloud;

  // Cast transform to float ONCE
  const Eigen::Matrix3f R = tf.rotation().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  detail::applyTransformInPlace(result, R, t);

  return result;
}

// Move version
template <typename Scalar, int Mode>
PointCloud transform(PointCloud&& cloud,
                     const Eigen::Transform<Scalar, 3, Mode>& tf) {
  if (cloud.empty()) {
    return std::move(cloud);
  }

  // Cast transform to float ONCE
  const Eigen::Matrix3f R = tf.rotation().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  detail::applyTransformInPlace(cloud, R, t);

  return std::move(cloud);
}

// ============================================================================
// Operator* Overloads (copy only)
// ============================================================================

template <typename Scalar>
inline PointCloud operator*(const Transform_<Scalar>& tf,
                            const PointCloud& cloud) {
  return transform(cloud, tf);
}

template <typename Scalar>
inline PointCloud operator*(const SE3_<Scalar>& se3, const PointCloud& cloud) {
  return transform(cloud, se3);
}

template <typename Scalar, int Mode>
inline PointCloud operator*(const Eigen::Transform<Scalar, 3, Mode>& tf,
                            const PointCloud& cloud) {
  return transform(cloud, tf);
}

}  // namespace nanopcl

#endif  // NANOPCL_TRANSFORM_IMPL_TRANSFORM_OPS_IMPL_HPP
