// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Point-to-Plane ICP registration.
//
// Point-to-Plane ICP minimizes point-to-plane distances, providing faster
// convergence and better accuracy on planar surfaces compared to
// Point-to-Point.
//
// Example usage:
//   // 1. Estimate normals on target (required)
//   geometry::estimateNormals(target, 0.3f);
//
//   // 2. Build search index
//   search::KdTree tree;
//   tree.build(target);
//
//   // 3. Run Point-to-Plane ICP
//   auto result = registration::icpPlane(source, target, tree);

#ifndef NANOPCL_REGISTRATION_ICP_PLANE_HPP
#define NANOPCL_REGISTRATION_ICP_PLANE_HPP

#include <stdexcept>

#include "nanopcl/registration/icp.hpp"
#include "nanopcl/registration/robust_kernels.hpp"

namespace npcl {
namespace registration {

/**
 * @brief Configuration for Point-to-Plane ICP
 *
 * Inherits from ICPConfig. Additional plane-specific parameters can be added.
 */
struct ICPPlaneConfig : ICPConfig {
  // Robust estimation (M-estimator)
  RobustKernel robust_kernel = RobustKernel::NONE;  ///< Robust kernel type
  double robust_kernel_width = 1.0;  ///< Kernel width (k parameter)
};

/**
 * @brief Point-to-Plane ICP with pre-built search index
 *
 * Minimizes point-to-plane distances for faster convergence on planar surfaces.
 * This is generally more robust than Point-to-Point ICP for structured environments.
 *
 * @pre Target cloud MUST have normals computed (geometry::estimateNormals).
 *
 * @tparam SearchMethod Search structure with nearest(Point, float) method.
 * @param source Source point cloud (to be aligned).
 * @param target Target point cloud (reference, must have normals).
 * @param target_search Pre-built search index for target.
 * @param initial_guess Initial transformation estimate.
 * @param config Algorithm configuration.
 * @return Registration result with transformation and quality metrics.
 *
 * @throws std::runtime_error if target has no normal channel.
 *
 * @code
 *   // 1. Prepare Target
 *   geometry::estimateNormals(target, 0.3f);
 *   search::KdTree tree;
 *   tree.build(target);
 *
 *   // 2. Run Registration
 *   auto result = registration::icpPlane(source, target, tree);
 * @endcode
 */
template <typename SearchMethod>
[[nodiscard]] RegistrationResult icpPlane(
    const PointCloud& source, const PointCloud& target,
    const SearchMethod& target_search,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const ICPPlaneConfig& config = ICPPlaneConfig{});

/**
 * @brief Point-to-Plane ICP (builds search index internally)
 *
 * Convenience overload that builds the search index internally.
 *
 * @tparam SearchMethod Search structure type (default: search::KdTree)
 * @throws std::runtime_error if target has no normals
 */
template <typename SearchMethod = search::KdTree>
[[nodiscard]] RegistrationResult icpPlane(
    const PointCloud& source, const PointCloud& target,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const ICPPlaneConfig& config = ICPPlaneConfig{});

}  // namespace registration
}  // namespace npcl

#include "nanopcl/registration/impl/icp_plane_impl.hpp"

#endif  // NANOPCL_REGISTRATION_ICP_PLANE_HPP
