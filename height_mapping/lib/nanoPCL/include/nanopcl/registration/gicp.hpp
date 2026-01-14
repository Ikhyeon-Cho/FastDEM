// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Generalized ICP (GICP) registration.
//
// GICP uses point covariances to achieve "plane-to-plane" registration,
// providing faster convergence and better accuracy than Point-to-Point ICP.
//
// Reference: Segal, Haehnel, Thrun. "Generalized-ICP" (RSS 2009)
//
// Example usage:
//   // 1. Estimate covariances on both clouds (required)
//   geometry::estimateCovariances(source, 0.3f);
//   geometry::estimateCovariances(target, 0.3f);
//
//   // 2. Build search index
//   search::KdTree tree;
//   tree.build(target);
//
//   // 3. Run GICP
//   auto result = registration::gicp(source, target, tree);

#ifndef NANOPCL_REGISTRATION_GICP_HPP
#define NANOPCL_REGISTRATION_GICP_HPP

#include <Eigen/Geometry>
#include <stdexcept>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/registration/correspondence.hpp"
#include "nanopcl/registration/result.hpp"
#include "nanopcl/registration/robust_kernels.hpp"
#include "nanopcl/search/kdtree.hpp"

namespace npcl {
namespace registration {

/**
 * @brief Configuration for GICP algorithm
 *
 * Controls convergence criteria, correspondence finding, and robustness.
 */
struct GICPConfig {
  /// Maximum number of iterations (Stop condition 1)
  int max_iterations = 50;

  /// Maximum distance to search for correspondences (meters)
  /// Points farther than this will be ignored.
  float max_correspondence_dist = 1.0f;

  /// Minimum number of valid correspondences to continue optimization
  /// If fewer points match, the algorithm aborts.
  size_t min_correspondences = 10;

  // --- Convergence Thresholds ---

  /// Stop if translation magnitude is smaller than this (meters)
  double translation_threshold = 1e-6;

  /// Stop if rotation angle is smaller than this (radians)
  double rotation_threshold = 1e-6;

  /// Stop if relative change in Mean Squared Error is smaller than this
  double relative_mse_threshold = 1e-6;

  // --- GICP Specifics ---

  /// Regularization constant for covariance matrices (epsilon)
  /// Prevents singularity when plane is perfectly flat.
  /// Replaces smallest eigenvalue with this value.
  double covariance_epsilon = 1e-3;

  // --- Robust Estimation ---

  /// M-Estimator kernel type to handle outliers
  /// Options: NONE, HUBER, CAUCHY, TUKEY
  RobustKernel robust_kernel = RobustKernel::NONE;

  /// Width parameter (k) for the robust kernel
  /// For Huber, residuals larger than k are downweighted.
  double robust_kernel_width = 1.0;
};

/**
 * @brief GICP registration with pre-built search index
 *
 * Performs Generalized-ICP to align source cloud to target cloud.
 * This variant assumes "Plane-to-Plane" metrics by utilizing covariance matrices.
 *
 * @pre Source cloud MUST have covariances computed (geometry::estimateCovariances)
 * @pre Target cloud MUST have covariances computed
 *
 * @tparam SearchMethod Search structure (e.g., search::KdTree, search::VoxelHash)
 * @param source Source point cloud (to be transformed)
 * @param target Target point cloud (reference)
 * @param target_search Pre-built search index for target cloud
 * @param initial_guess Initial transformation estimate (default: Identity)
 * @param config Algorithm configuration parameters
 * @return RegistrationResult containing final transform, fitness, and RMSE
 *
 * @throws std::runtime_error if source or target does not have covariance channel
 *
 * @code
 *   // 1. Prepare Data
 *   geometry::estimateCovariances(source, 0.5f);
 *   geometry::estimateCovariances(target, 0.5f);
 *
 *   search::KdTree tree;
 *   tree.build(target);
 *
 *   // 2. Run GICP
 *   GICPConfig config;
 *   config.max_correspondence_dist = 0.5f;
 *   auto result = registration::gicp(source, target, tree, initial_guess, config);
 *
 *   if (result.converged) {
 *     std::cout << "Aligned! Score: " << result.fitness << "\n";
 *   }
 * @endcode
 */
template <typename SearchMethod>
[[nodiscard]] RegistrationResult gicp(
    const PointCloud& source, const PointCloud& target,
    const SearchMethod& target_search,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const GICPConfig& config = GICPConfig{});

/**
 * @brief GICP registration (builds search index internally)
 *
 * Convenience overload that builds the search index internally.
 *
 * @tparam SearchMethod Search structure type (default: search::KdTree)
 * @throws std::runtime_error if source or target missing covariances
 */
template <typename SearchMethod = search::KdTree>
[[nodiscard]] RegistrationResult gicp(
    const PointCloud& source, const PointCloud& target,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const GICPConfig& config = GICPConfig{});

}  // namespace registration
}  // namespace npcl

#include "nanopcl/registration/impl/gicp_impl.hpp"

#endif  // NANOPCL_REGISTRATION_GICP_HPP
