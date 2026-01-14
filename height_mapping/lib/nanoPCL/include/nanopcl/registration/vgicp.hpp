// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Voxelized Generalized ICP (VGICP) registration.
//
// VGICP combines the accuracy of GICP with the speed of voxel-based methods.
// Instead of per-point covariances on the target, it uses pre-computed
// voxel distributions for O(1) correspondence lookup.
//
// Key advantages over standard GICP:
//   - No k-NN search required (O(1) voxel lookup vs O(log N) tree search)
//   - Pre-computed voxel distributions amortize covariance calculation
//   - Cache-friendly memory access pattern
//
// Reference: Koide et al., "Voxelized GICP for Fast and Accurate 3D Point
//            Cloud Registration" (ICRA 2021)
//
// Example usage:
//   // 1. Build voxel map from target (one-time cost)
//   registration::VoxelDistributionMap voxel_map(0.5f);
//   voxel_map.build(target);
//
//   // 2. Estimate covariances on source
//   geometry::estimateCovariances(source, 0.3f);
//
//   // 3. Run VGICP
//   auto result = registration::vgicp(source, voxel_map);

#ifndef NANOPCL_REGISTRATION_VGICP_HPP
#define NANOPCL_REGISTRATION_VGICP_HPP

#include <Eigen/Geometry>
#include <stdexcept>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/registration/correspondence.hpp"
#include "nanopcl/registration/result.hpp"
#include "nanopcl/registration/robust_kernels.hpp"
#include "nanopcl/registration/voxel_distribution_map.hpp"

namespace npcl {
namespace registration {

/**
 * @brief Configuration for VGICP algorithm
 *
 * Default thresholds are relaxed compared to point-based ICP because
 * VGICP matches to voxel centroids (discrete approximation) rather than
 * exact nearest neighbors.
 */
struct VGICPConfig {
  /// Maximum number of iterations (Fewer needed with voxels)
  int max_iterations = 30;

  /// Minimum correspondences to continue
  size_t min_correspondences = 10;

  // --- Convergence Thresholds ---

  /// Translation magnitude threshold (0.1mm)
  double translation_threshold = 1e-4;

  /// Rotation angle threshold (~0.006°)
  double rotation_threshold = 1e-4;

  /// Relative MSE change threshold (0.01%)
  double relative_mse_threshold = 1e-4;

  // --- GICP Specifics ---

  /// Regularization for source covariance
  double covariance_epsilon = 1e-3;

  // --- Robust Estimation ---

  /// Robust kernel type
  RobustKernel robust_kernel = RobustKernel::NONE;

  /// Kernel width (k parameter)
  double robust_kernel_width = 1.0;
};

/**
 * @brief VGICP registration with pre-built voxel map
 *
 * Uses Mahalanobis distance weighting with voxel-based target distributions.
 * Source cloud must have covariances; target distributions are from voxel map.
 *
 * @param source Source point cloud (MUST have covariances)
 * @param voxel_map Pre-built voxel distribution map for target
 * @param initial_guess Initial transformation estimate (default: identity)
 * @param config Algorithm configuration
 * @return Registration result with transformation and quality metrics
 *
 * @throws std::runtime_error if source missing covariances
 *
 * @code
 *   // 1. One-time Setup (Target)
 *   registration::VoxelDistributionMap voxel_map(0.5f); // 0.5m voxel
 *   voxel_map.build(target);
 *
 *   // 2. Per-scan Setup (Source)
 *   geometry::estimateCovariances(source, 0.3f);
 *
 *   // 3. Register
 *   auto result = registration::vgicp(source, voxel_map);
 * @endcode
 */
[[nodiscard]] RegistrationResult vgicp(
    const PointCloud& source, const VoxelDistributionMap& voxel_map,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const VGICPConfig& config = VGICPConfig{});

/**
 * @brief VGICP registration (builds voxel map internally)
 *
 * Convenience overload that builds the voxel map from target cloud.
 * For repeated registrations to the same target, pre-build the voxel map.
 *
 * @param source Source point cloud (MUST have covariances)
 * @param target Target point cloud
 * @param voxel_resolution Voxel size for target distribution map
 * @param initial_guess Initial transformation estimate
 * @param config Algorithm configuration
 * @return Registration result with transformation and quality metrics
 *
 * @throws std::runtime_error if source missing covariances
 */
[[nodiscard]] RegistrationResult vgicp(
    const PointCloud& source, const PointCloud& target, float voxel_resolution,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const VGICPConfig& config = VGICPConfig{});

}  // namespace registration
}  // namespace npcl

#include "nanopcl/registration/impl/vgicp_impl.hpp"

#endif  // NANOPCL_REGISTRATION_VGICP_HPP
