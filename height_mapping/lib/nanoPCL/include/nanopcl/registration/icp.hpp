// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Point-to-Point ICP registration.
//
// Example usage:
//   search::KdTree tree;
//   tree.build(target);
//   auto result = registration::icp(source, target, tree);
//
//   // Or with VoxelHash for real-time applications:
//   search::VoxelHash hash(0.5f);
//   hash.build(target);
//   auto result = registration::icp(source, target, hash);

#ifndef NANOPCL_REGISTRATION_ICP_HPP
#define NANOPCL_REGISTRATION_ICP_HPP

#include <Eigen/Geometry>
#include <limits>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/registration/correspondence.hpp"
#include "nanopcl/registration/result.hpp"
#include "nanopcl/search/kdtree.hpp"

namespace npcl {
namespace registration {

/**
 * @brief ICP algorithm configuration
 */
struct ICPConfig {
  /// Maximum number of iterations
  int max_iterations = 50;

  /// Maximum distance for valid correspondence (meters)
  /// Points farther than this will be ignored.
  float max_correspondence_dist = 1.0f;

  /// Minimum correspondences to continue optimization
  size_t min_correspondences = 10;

  // --- Convergence Thresholds ---

  /// Stop if translation magnitude is smaller than this (meters)
  double translation_threshold = 1e-6;

  /// Stop if rotation angle is smaller than this (radians)
  double rotation_threshold = 1e-6;

  /// Stop if relative MSE change is smaller than this (fallback)
  double relative_mse_threshold = 1e-6;
};

/**
 * @brief Point-to-Point ICP with pre-built search index
 *
 * Finds the rigid transformation that aligns source cloud to target cloud
 * by minimizing point-to-point distances.
 *
 * @tparam SearchMethod Search structure with signature:
 *                      `std::optional<NearestResult> nearest(const Point&, float)`
 *                      (e.g., search::KdTree, search::VoxelHash)
 * @param source Source point cloud (to be aligned)
 * @param target Target point cloud (reference)
 * @param target_search Pre-built search index for target cloud
 * @param initial_guess Initial transformation estimate (default: identity)
 * @param config Algorithm configuration
 * @return Registration result with transformation and quality metrics
 *
 * @note For repeated registration to the same target, pre-build the search
 *       index once and reuse it for better performance.
 */
template <typename SearchMethod>
[[nodiscard]] RegistrationResult icp(
    const PointCloud& source, const PointCloud& target,
    const SearchMethod& target_search,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const ICPConfig& config = ICPConfig{});

/**
 * @brief Point-to-Point ICP (builds search index internally)
 *
 * Convenience overload that builds the search index internally.
 * Use the overload with pre-built search index for repeated registrations.
 *
 * @tparam SearchMethod Search structure type (default: search::KdTree)
 */
template <typename SearchMethod = search::KdTree>
[[nodiscard]] RegistrationResult icp(
    const PointCloud& source, const PointCloud& target,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const ICPConfig& config = ICPConfig{});

}  // namespace registration
}  // namespace npcl

#include "nanopcl/registration/impl/icp_impl.hpp"

#endif  // NANOPCL_REGISTRATION_ICP_HPP
