// nanopcl/transform/transform_ops.hpp
// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_TRANSFORM_TRANSFORM_OPS_HPP
#define NANOPCL_TRANSFORM_TRANSFORM_OPS_HPP

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/transform/transform.hpp"

namespace nanopcl {

// ============================================================================
// PointCloud Transformation with Frame-aware Transform
// ============================================================================

/**
 * @brief Transform PointCloud (copy version)
 * @param cloud Input point cloud (unchanged)
 * @param tf Transform from cloud's frame to target frame
 * @return Transformed point cloud in target frame
 * @throws std::runtime_error if frame mismatch or transform not initialized
 *
 * @note Optimized: casts transform to float once, all point ops in float.
 */
template <typename Scalar>
PointCloud transform(const PointCloud& cloud, const Transform_<Scalar>& tf);

/**
 * @brief Transform PointCloud (move version - optimal performance)
 * @param cloud Input point cloud (consumed, do not use after call)
 * @param tf Transform from cloud's frame to target frame
 * @return Transformed point cloud in target frame (reuses input memory)
 * @throws std::runtime_error if frame mismatch or transform not initialized
 *
 * @note Use std::move() to enable this optimization.
 *
 * Example:
 * @code
 * cloud = transform(std::move(cloud), T_map_base);
 * @endcode
 */
template <typename Scalar>
PointCloud transform(PointCloud&& cloud, const Transform_<Scalar>& tf);

// ============================================================================
// SE3 Transform API
// ============================================================================

/**
 * @brief Transform PointCloud with SE3 (copy version)
 * @param cloud Input point cloud (unchanged)
 * @param se3 SE3 transformation to apply
 * @return Transformed point cloud
 *
 * @note Frame ID is preserved (SE3 is frameless)
 */
template <typename Scalar>
PointCloud transform(const PointCloud& cloud, const SE3_<Scalar>& se3);

/**
 * @brief Transform PointCloud with SE3 (move version)
 */
template <typename Scalar>
PointCloud transform(PointCloud&& cloud, const SE3_<Scalar>& se3);

// ============================================================================
// Eigen::Transform API (Isometry, Affine)
// ============================================================================

/**
 * @brief Transform PointCloud with Eigen::Transform (copy version)
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
PointCloud transform(const PointCloud& cloud,
                     const Eigen::Transform<Scalar, 3, Mode>& tf);

/**
 * @brief Transform PointCloud with Eigen::Transform (move version)
 */
template <typename Scalar, int Mode>
PointCloud transform(PointCloud&& cloud,
                     const Eigen::Transform<Scalar, 3, Mode>& tf);

// ============================================================================
// Operator* syntax sugar (copy only)
// ============================================================================

/**
 * @brief Operator syntax for transforming PointCloud with frame-aware Transform
 * @note Enables: auto cloud_map = T_map_laser * cloud_laser;
 * @note Always returns a copy. Use transform(std::move(cloud), tf) for move.
 */
template <typename Scalar>
PointCloud operator*(const Transform_<Scalar>& tf, const PointCloud& cloud);

/**
 * @brief Operator syntax for transforming PointCloud with SE3
 * @note Enables: auto cloud_out = se3 * cloud_in; (frameless)
 */
template <typename Scalar>
PointCloud operator*(const SE3_<Scalar>& se3, const PointCloud& cloud);

/**
 * @brief Operator syntax for Eigen::Transform * PointCloud
 * @note Enables: auto cloud_map = T_map_base * cloud_base;
 *       where T_map_base is Eigen::Isometry3d, Affine3f, etc.
 */
template <typename Scalar, int Mode>
PointCloud operator*(const Eigen::Transform<Scalar, 3, Mode>& tf,
                     const PointCloud& cloud);

}  // namespace nanopcl

// Include implementation
#include "nanopcl/transform/impl/transform_ops_impl.hpp"

#endif  // NANOPCL_TRANSFORM_TRANSFORM_OPS_HPP
