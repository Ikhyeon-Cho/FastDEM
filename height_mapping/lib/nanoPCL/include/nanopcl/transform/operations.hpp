// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_TRANSFORM_OPERATIONS_HPP
#define NANOPCL_TRANSFORM_OPERATIONS_HPP

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/transform/transform.hpp"

namespace nanopcl {

// ============================================================================
// PointCloud Transformation
// ============================================================================

/**
 * @brief Transform PointCloud to new frame (optimized implementation)
 * @param cloud Point cloud in source frame
 * @param tf Transform from cloud's frame to target frame
 * @return Transformed point cloud in target frame
 * @throws std::runtime_error if frame mismatch
 *
 * @note Optimized: casts transform to float once, all point ops in float.
 * @note Return-by-value is efficient: C++17 copy elision guarantees zero overhead.
 */
template <typename Scalar>
PointCloud transform(const PointCloud& cloud, const Transform_<Scalar>& tf) {
  assert(tf.isValid() && "Transform not initialized");
  if (tf.source_frame() != cloud.frameId()) {
    throw std::runtime_error(
        "transform(PointCloud, Transform): frame mismatch: transform.from='" +
        tf.source_frame() + "' != cloud.frameId='" + cloud.frameId() + "'");
  }

  // Early return for empty clouds
  if (cloud.empty()) {
    PointCloud result;
    result.setFrameId(tf.target_frame());
    result.setTimestamp(cloud.timestamp());
    return result;
  }

  // Deep copy (preserves all channels)
  PointCloud result = cloud;

  // Cast transform to float ONCE (optimal: "Points float, Poses double")
  const Eigen::Matrix3f R = tf.rotation().matrix().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  // All point operations in float (SIMD optimized)
  for (auto& point : result.points) {
    point = R * point + t;
  }

  // Update metadata
  result.setFrameId(tf.target_frame());

  return result;
}

/**
 * @brief Transform PointCloud in-place
 * @param cloud Point cloud to transform (modified in-place)
 * @param tf Transform to apply
 * @throws std::runtime_error if frame mismatch
 *
 * @note More memory efficient than transform() for large clouds
 * @note Optimized: casts transform to float once, all point ops in float.
 */
template <typename Scalar>
void transformInPlace(PointCloud& cloud, const Transform_<Scalar>& tf) {
  assert(tf.isValid() && "Transform not initialized");
  if (tf.source_frame() != cloud.frameId()) {
    throw std::runtime_error(
        "transformInPlace: frame mismatch: transform.from='" +
        tf.source_frame() + "' != cloud.frameId='" + cloud.frameId() + "'");
  }

  if (cloud.empty()) return;

  // Cast transform to float ONCE
  const Eigen::Matrix3f R = tf.rotation().matrix().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  // All point operations in float
  for (auto& point : cloud.points) {
    point = R * point + t;
  }

  cloud.setFrameId(tf.target_frame());
}

// ============================================================================
// Generic Transform API (accepts any Eigen-compatible transform)
// ============================================================================

/**
 * @brief Transform PointCloud with Eigen::Transform (Isometry, Affine, etc.)
 *
 * @tparam Scalar float or double
 * @tparam Mode Eigen::Isometry, Eigen::Affine, etc.
 *
 * This is the recommended API for SLAM/robotics applications:
 * - Users keep trajectories in double precision (Isometry3d)
 * - Pass directly to this function (no manual casting)
 * - Internal computation uses float (SIMD optimized)
 *
 * @note "Points are float, Poses are double" - robotics de facto standard
 */
template <typename Scalar, int Mode>
void transformInPlace(PointCloud& cloud,
                      const Eigen::Transform<Scalar, 3, Mode>& tf) {
  if (cloud.empty()) return;

  // Cast transform to float ONCE (4x4 matrix = trivial cost)
  const Eigen::Matrix3f R = tf.rotation().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  // Apply to all points in float (SIMD optimized)
  for (auto& point : cloud.points) {
    point = R * point + t;
  }
}

/**
 * @brief Transform PointCloud with 4x4 matrix
 *
 * @tparam Derived Eigen matrix expression type
 *
 * Accepts any 4x4 matrix: Matrix4f, Matrix4d, expressions, etc.
 */
template <typename Derived>
std::enable_if_t<Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4>
transformInPlace(PointCloud& cloud, const Eigen::MatrixBase<Derived>& tf_matrix) {
  if (cloud.empty()) return;

  // Cast to float ONCE
  const Eigen::Matrix4f tf_f = tf_matrix.template cast<float>();
  const Eigen::Matrix3f R = tf_f.template block<3, 3>(0, 0);
  const Eigen::Vector3f t = tf_f.template block<3, 1>(0, 3);

  // Apply to all points in float
  for (auto& point : cloud.points) {
    point = R * point + t;
  }
}

/**
 * @brief Transform PointCloud with Eigen::Transform (returns new cloud)
 */
template <typename Scalar, int Mode>
PointCloud transform(const PointCloud& cloud,
                     const Eigen::Transform<Scalar, 3, Mode>& tf) {
  PointCloud result = cloud;  // Deep copy
  transformInPlace(result, tf);
  return result;
}

/**
 * @brief Transform PointCloud with 4x4 matrix (returns new cloud)
 */
template <typename Derived>
std::enable_if_t<Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
                 PointCloud>
transform(const PointCloud& cloud, const Eigen::MatrixBase<Derived>& tf_matrix) {
  PointCloud result = cloud;
  transformInPlace(result, tf_matrix);
  return result;
}

// ============================================================================
// Operator* syntax sugar
// ============================================================================

/**
 * @brief Operator syntax for transforming PointCloud with frame-aware Transform
 * @note Enables: auto cloud_map = T_map_laser * cloud_laser;
 *
 * Optimized: casts transform matrix to float once.
 */
template <typename Scalar>
inline PointCloud operator*(const Transform_<Scalar>& tf,
                            const PointCloud& cloud) {
  PointCloud result = cloud;  // Deep copy preserves all channels

  // Cast to float ONCE
  const Eigen::Matrix3f R = tf.rotation().matrix().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  // All point operations in float
  for (auto& point : result.points) {
    point = R * point + t;
  }

  result.setFrameId(tf.target_frame());
  return result;
}

/**
 * @brief Operator syntax for transforming PointCloud with SE3
 * @note Enables: auto cloud_out = se3 * cloud_in; (frameless)
 *
 * Optimized: casts SE3 matrix to float once, then all point ops in float.
 */
template <typename Scalar>
inline PointCloud operator*(const SE3_<Scalar>& se3, const PointCloud& cloud) {
  PointCloud result;
  result.reserve(cloud.size());
  result.resize(cloud.size());

  // Preserve all enabled channels
  if (cloud.hasIntensity()) {
    result.enableIntensity();
    result.intensity() = cloud.intensity();
  }
  if (cloud.hasTime()) {
    result.enableTime();
    result.time() = cloud.time();
  }
  if (cloud.hasRing()) {
    result.enableRing();
    result.ring() = cloud.ring();
  }
  if (cloud.hasColor()) {
    result.enableColor();
    result.color() = cloud.color();
  }
  if (cloud.hasLabel()) {
    result.enableLabel();
    result.label() = cloud.label();
  }

  // Cast SE3 to float ONCE (optimal: "Points float, Poses double")
  const Eigen::Matrix3f R = se3.rotation().matrix().template cast<float>();
  const Eigen::Vector3f t = se3.translation().template cast<float>();

  // All point operations in float (SIMD optimized)
  for (size_t i = 0; i < cloud.size(); ++i) {
    result.points[i] = R * cloud.points[i] + t;
  }

  result.setFrameId(cloud.frameId());
  result.setTimestamp(cloud.timestamp());
  return result;
}

// ============================================================================
// Operator* for Eigen types
// ============================================================================

/**
 * @brief Operator syntax for Eigen::Transform * PointCloud
 * @note Enables: auto cloud_map = T_map_base * cloud_base;
 *       where T_map_base is Eigen::Isometry3d, Affine3f, etc.
 */
template <typename Scalar, int Mode>
inline PointCloud operator*(const Eigen::Transform<Scalar, 3, Mode>& tf,
                            const PointCloud& cloud) {
  return transform(cloud, tf);
}

/**
 * @brief Operator syntax for Eigen::Matrix4 * PointCloud
 */
template <typename Derived>
inline std::enable_if_t<
    Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
    PointCloud>
operator*(const Eigen::MatrixBase<Derived>& tf_matrix, const PointCloud& cloud) {
  return transform(cloud, tf_matrix);
}

}  // namespace nanopcl

#endif  // NANOPCL_TRANSFORM_OPERATIONS_HPP
