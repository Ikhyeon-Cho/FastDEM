// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Plane Segmentation using RANSAC
//
// Fast plane fitting optimized for ground plane removal in robotics.
// Uses OpenMP parallelization for inlier counting.
//
// Example usage:
//   auto result = segmentation::segmentPlane(cloud, 0.1f);  // 10cm threshold
//   if (result.success()) {
//     auto ground = cloud[result.inliers];
//     auto obstacles = cloud[result.outliers(cloud.size())];
//   }

#ifndef NANOPCL_SEGMENTATION_RANSAC_PLANE_HPP
#define NANOPCL_SEGMENTATION_RANSAC_PLANE_HPP

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace npcl {
namespace segmentation {

// =============================================================================
// Plane Model (Hessian Normal Form)
// =============================================================================

/**
 * @brief Plane model in Hessian Normal Form: n·p + d = 0
 *
 * The normal is unit length, making d the signed distance from origin.
 * Signed distance from any point p: n·p + d
 */
struct PlaneModel {
  Eigen::Vector3f normal{0.f, 0.f, 1.f};  ///< Unit normal vector
  float d{0.f};                           ///< Signed distance from origin

  /**
   * @brief Compute signed distance from point to plane
   */
  [[nodiscard]] float signedDistance(const Point& p) const noexcept {
    return normal.dot(p) + d;
  }

  /**
   * @brief Compute absolute distance from point to plane
   */
  [[nodiscard]] float distance(const Point& p) const noexcept {
    return std::abs(signedDistance(p));
  }

  /**
   * @brief Create plane from 3 points
   * @return Plane model, or invalid plane if points are collinear
   */
  [[nodiscard]] static PlaneModel fromPoints(const Point& p1, const Point& p2,
                                             const Point& p3);

  /**
   * @brief Check if plane is valid
   */
  [[nodiscard]] bool isValid() const noexcept { return std::isfinite(d); }
};

// =============================================================================
// RANSAC Result
// =============================================================================

/**
 * @brief Result of RANSAC plane segmentation
 */
struct RansacResult {
  PlaneModel model;               ///< Best plane model found
  std::vector<uint32_t> inliers;  ///< Indices of inlier points
  double fitness{0.0};            ///< Inlier ratio (0.0 - 1.0)
  int iterations{0};              ///< Number of iterations performed

  /**
   * @brief Get outlier indices (complement of inliers)
   * @param total_points Total number of points in the cloud
   */
  [[nodiscard]] std::vector<uint32_t> outliers(size_t total_points) const;

  /**
   * @brief Check if segmentation was successful
   */
  [[nodiscard]] bool success() const noexcept {
    return model.isValid() && !inliers.empty();
  }
};

// =============================================================================
// RANSAC Configuration
// =============================================================================

/**
 * @brief Configuration for RANSAC algorithm
 */
struct RansacConfig {
  float distance_threshold{0.1f};  ///< Max distance to be considered inlier (m)
  int max_iterations{1000};        ///< Maximum RANSAC iterations
  double probability{0.99};  ///< Desired probability of finding best model
  size_t min_inliers{3};     ///< Minimum inliers to accept model
  bool refine_model{true};   ///< Refine plane using all inliers (SVD)
};

// =============================================================================
// Plane Segmentation API
// =============================================================================

/**
 * @brief Segment the largest plane using RANSAC
 *
 * Uses OpenMP for parallel inlier counting and adaptive iteration.
 *
 * @param cloud Input point cloud
 * @param indices Subset of indices to consider
 * @param config RANSAC configuration
 * @return Segmentation result with plane model and inlier indices
 */
[[nodiscard]] RansacResult segmentPlane(
    const PointCloud& cloud, const std::vector<uint32_t>& indices,
    const RansacConfig& config = RansacConfig{});

/**
 * @brief Segment the largest plane using RANSAC (full cloud version)
 *
 * Convenience overload that processes all points in the cloud.
 *
 * @param cloud Input point cloud
 * @param distance_threshold Max distance for inliers (meters)
 * @param max_iterations Maximum RANSAC iterations
 * @param probability Desired success probability
 * @return Segmentation result with plane model and inlier indices
 */
[[nodiscard]] RansacResult segmentPlane(const PointCloud& cloud,
                                        float distance_threshold,
                                        int max_iterations = 1000,
                                        double probability = 0.99);

/**
 * @brief Extract multiple planes iteratively
 *
 * Repeatedly applies RANSAC to find multiple planes.
 * Useful for structured environments (walls, floor, ceiling).
 *
 * @param cloud Input point cloud
 * @param config RANSAC configuration
 * @param max_planes Maximum number of planes to extract
 * @param min_fitness Minimum inlier ratio to accept a plane
 * @return Vector of segmentation results (largest first)
 */
[[nodiscard]] std::vector<RansacResult> segmentMultiplePlanes(
    const PointCloud& cloud, const RansacConfig& config = RansacConfig{},
    size_t max_planes = 5, double min_fitness = 0.1);

}  // namespace segmentation
}  // namespace npcl

#include "nanopcl/segmentation/impl/ransac_plane_impl.hpp"

#endif  // NANOPCL_SEGMENTATION_RANSAC_PLANE_HPP
