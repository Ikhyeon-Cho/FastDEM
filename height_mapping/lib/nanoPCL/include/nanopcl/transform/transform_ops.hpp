// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// PointCloud transformation operations.

#ifndef NANOPCL_TRANSFORM_TRANSFORM_OPS_HPP
#define NANOPCL_TRANSFORM_TRANSFORM_OPS_HPP

#include <stdexcept>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/transform/transform.hpp"

namespace npcl {

namespace detail {

/**
 * @brief Apply R*p + t to all points in-place
 *
 * PERFORMANCE NOTE (benchmarked 2026-01):
 * This simple per-point loop is faster than Eigen::Map batch processing.
 *
 * Tested alternatives that were SLOWER:
 * 1. Eigen::Map<Matrix3Xf> + batch GEMM: 5x slower (500K points)
 *    - Reason: Temporary 3xN matrix allocation, multiple memory passes
 * 2. Pre-allocated temporary matrix: Same as above
 *
 * Why current approach is optimal:
 * - Single pass over memory (cache-friendly)
 * - CPU prefetcher handles sequential access well
 * - Eigen expression templates already optimize R*p+t
 * - No heap allocation
 * - Works well in both warm and cold cache scenarios
 *
 * Benchmark: 500K points, -O3 -march=native, AVX enabled
 *   Per-point loop:    0.43ms (1.00x) - BEST
 *   Eigen::Map batch:  2.32ms (0.19x) - 5x slower
 *
 * See: benchmarks/profiling/benchmark_transform.cpp
 */
inline void applyTransformInPlace(PointCloud& cloud, const Eigen::Matrix3f& R,
                                  const Eigen::Vector3f& t) {
  for (auto& point : cloud.xyz()) {
    point = R * point + t;
  }

  // Transform normals if present (rotation only, no translation)
  if (cloud.hasNormal()) {
    for (auto& n : cloud.normal()) {
      n = R * n;
    }
  }
}

}  // namespace detail

// =============================================================================
// Frame-aware Transform API
// =============================================================================

/**
 * @brief Transform PointCloud using frame-aware Transform (copy version)
 *
 * @param cloud Input point cloud (unchanged)
 * @param tf Transform from cloud's frame to target frame
 * @return Transformed point cloud in target frame
 * @throws std::runtime_error if frame mismatch or transform not initialized
 *
 * @note Validates that cloud.frameId() matches tf.childFrame()
 */
template <typename Scalar>
[[nodiscard]] PointCloud transformCloud(const PointCloud& cloud,
                                        const Transform_<Scalar>& tf) {
  if (!tf.isValid()) {
    throw std::runtime_error("transformCloud: Transform not initialized");
  }
  if (tf.childFrame() != cloud.frameId()) {
    throw std::runtime_error(
        "transformCloud: frame mismatch: transform.child='" + tf.childFrame() +
        "' != cloud.frameId='" + cloud.frameId() + "'");
  }

  // Early return for empty clouds
  if (cloud.empty()) {
    PointCloud result;
    result.setFrameId(tf.parentFrame());
    result.setTimestamp(cloud.timestamp());
    return result;
  }

  // Deep copy (preserves all channels)
  PointCloud result = cloud;

  // Cast transform to float ONCE (optimal: "Points float, Poses double")
  const Eigen::Matrix3f R = tf.rotation().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  detail::applyTransformInPlace(result, R, t);
  result.setFrameId(tf.parentFrame());

  return result;
}

/**
 * @brief Transform PointCloud using frame-aware Transform (move version)
 *
 * @param cloud Input point cloud (consumed, do not use after call)
 * @param tf Transform from cloud's frame to target frame
 * @return Transformed point cloud in target frame (reuses input memory)
 * @throws std::runtime_error if frame mismatch or transform not initialized
 *
 * @note Use std::move() to enable this optimization:
 * @code
 * cloud = transformCloud(std::move(cloud), tf);
 * @endcode
 */
template <typename Scalar>
[[nodiscard]] PointCloud transformCloud(PointCloud&& cloud,
                                        const Transform_<Scalar>& tf) {
  if (!tf.isValid()) {
    throw std::runtime_error("transformCloud: Transform not initialized");
  }
  if (tf.childFrame() != cloud.frameId()) {
    throw std::runtime_error(
        "transformCloud: frame mismatch: transform.child='" + tf.childFrame() +
        "' != cloud.frameId='" + cloud.frameId() + "'");
  }

  if (cloud.empty()) {
    cloud.setFrameId(tf.parentFrame());
    return std::move(cloud);
  }

  // Cast transform to float ONCE
  const Eigen::Matrix3f R = tf.rotation().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  detail::applyTransformInPlace(cloud, R, t);
  cloud.setFrameId(tf.parentFrame());

  return std::move(cloud);
}

// =============================================================================
// Frameless Eigen::Transform API
// =============================================================================

/**
 * @brief Transform PointCloud with Eigen::Transform (copy version, frameless)
 *
 * @tparam Scalar float or double
 * @tparam Mode Eigen::Isometry, Eigen::Affine, etc.
 * @param cloud Input point cloud (unchanged)
 * @param tf Eigen transform to apply
 * @return Transformed point cloud (frameId preserved)
 *
 * @note "Points are float, Poses are double" - robotics de facto standard
 * @note This function does NOT modify the cloud's frameId
 */
template <typename Scalar, int Mode>
[[nodiscard]] PointCloud transformCloud(
    const PointCloud& cloud, const Eigen::Transform<Scalar, 3, Mode>& tf) {
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

/**
 * @brief Transform PointCloud with Eigen::Transform (move version, frameless)
 */
template <typename Scalar, int Mode>
[[nodiscard]] PointCloud transformCloud(
    PointCloud&& cloud, const Eigen::Transform<Scalar, 3, Mode>& tf) {
  if (cloud.empty()) {
    return std::move(cloud);
  }

  // Cast transform to float ONCE
  const Eigen::Matrix3f R = tf.rotation().template cast<float>();
  const Eigen::Vector3f t = tf.translation().template cast<float>();

  detail::applyTransformInPlace(cloud, R, t);

  return std::move(cloud);
}

}  // namespace npcl

#endif  // NANOPCL_TRANSFORM_TRANSFORM_OPS_HPP
