// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_POINT_HPP
#define NANOPCL_CORE_POINT_HPP

#include <Eigen/Core>

namespace nanopcl {

// ============================================================================
// Point Type Definition
// ============================================================================

/**
 * @brief 3D point with float precision
 *
 * Direct alias for Eigen::Vector3f, providing memory efficiency and cache
 * performance. Float precision is sufficient for typical robotics sensors
 * (LiDAR has ~3cm precision, cameras have pixel-level precision).
 *
 * Usage:
 * @code
 * Point p1(1.0f, 2.0f, 3.0f);
 * Point p2 = Point::Zero();
 * float dist = distance(p1, p2);
 * @endcode
 *
 * @note For double precision, use Eigen::Vector3d directly
 * @note This is NOT a template - Point is always float
 */
using Point = Eigen::Vector3f;

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Compute 3D Euclidean distance between two points
 * @param a First point
 * @param b Second point
 * @return Distance in meters
 */
inline float distance(const Point& a, const Point& b) {
  return (a - b).norm();
}

/**
 * @brief Compute 2D Euclidean distance between two points (ignoring z)
 * @param a First point
 * @param b Second point
 * @return 2D distance in meters
 */
inline float distance2D(const Point& a, const Point& b) {
  return (a.head<2>() - b.head<2>()).norm();
}

/**
 * @brief Compute squared 3D Euclidean distance (faster, no sqrt)
 * @param a First point
 * @param b Second point
 * @return Squared distance
 *
 * @note Use this for distance comparisons to avoid expensive sqrt()
 */
inline float distanceSquared(const Point& a, const Point& b) {
  return (a - b).squaredNorm();
}

/**
 * @brief Compute squared 2D Euclidean distance (faster, no sqrt)
 * @param a First point
 * @param b Second point
 * @return Squared 2D distance
 *
 * @note Use this for 2D distance comparisons to avoid expensive sqrt()
 */
inline float distance2DSquared(const Point& a, const Point& b) {
  return (a.head<2>() - b.head<2>()).squaredNorm();
}

}  // namespace nanopcl

#endif  // NANOPCL_CORE_POINT_HPP
