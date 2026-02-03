// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Convenience functions for point cloud registration.
// For advanced usage, use IterativeSolver directly.

#ifndef NANOPCL_REGISTRATION_ALIGN_HPP
#define NANOPCL_REGISTRATION_ALIGN_HPP

#include <Eigen/Geometry>
#include <stdexcept>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/registration/context.hpp"
#include "nanopcl/registration/correspondence/kdtree_correspondence.hpp"
#include "nanopcl/registration/correspondence/voxel_correspondence.hpp"
#include "nanopcl/registration/factors/gicp_factor.hpp"
#include "nanopcl/registration/factors/icp_factor.hpp"
#include "nanopcl/registration/factors/plane_factor.hpp"
#include "nanopcl/registration/factors/vgicp_factor.hpp"
#include "nanopcl/registration/iterative_solver.hpp"
#include "nanopcl/registration/result.hpp"

namespace nanopcl {
namespace registration {

// =============================================================================
// Registration Settings
// =============================================================================

/// @brief Common settings for all registration algorithms
struct AlignSettings {
  /// Maximum correspondence distance (meters)
  float max_correspondence_dist = 1.0f;

  /// Maximum iterations
  int max_iterations = 50;

  /// Translation convergence threshold (meters)
  double translation_eps = 1e-4;

  /// Rotation convergence threshold (radians)
  double rotation_eps = 1e-4;

  /// Relative error convergence threshold
  double relative_error_eps = 1e-6;

  /// Minimum correspondences required
  size_t min_correspondences = 10;

  /// Robust kernel type (for PlaneICP and GICP)
  RobustKernel robust_kernel = RobustKernel::NONE;

  /// Robust kernel width
  double robust_kernel_width = 1.0;

  /// Covariance regularization epsilon (for GICP)
  double covariance_epsilon = 1e-3;
};

// =============================================================================
// Point-to-Point ICP
// =============================================================================

/// @brief Align using Point-to-Point ICP
/// @param source Source point cloud
/// @param target Target point cloud
/// @param initial_guess Initial transformation estimate
/// @param settings Registration settings
/// @return Registration result
[[nodiscard]] inline RegistrationResult alignICP(
    const PointCloud& source,
    const PointCloud& target,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const AlignSettings& settings = {}) {
  if (source.empty() || target.empty()) {
    return {initial_guess, 0.0, std::numeric_limits<double>::infinity(), 0, false, std::nullopt};
  }

  // Build correspondence finder
  KdTreeCorrespondence correspondence;
  correspondence.build(target);

  const size_t n = source.size();
  const float max_dist_sq =
      settings.max_correspondence_dist * settings.max_correspondence_dist;
  ICPFactor::Setting factor_setting;

  // Configure solver
  IterativeSolver<> solver;
  solver.setMaxIterations(settings.max_iterations);
  solver.setConvergenceThresholds(settings.translation_eps, settings.rotation_eps);
  solver.setMinCorrespondences(settings.min_correspondences);
  solver.criteria().relative_error_eps = settings.relative_error_eps;

  return solver.solve(
      n,
      [&](size_t idx, const Eigen::Isometry3d& T, double* H, double* b, double* e) {
        // Transform source point
        const Eigen::Vector3d p_src = T * source[idx].head<3>().cast<double>();

        // Find correspondence
        auto ctx = correspondence.find(p_src, max_dist_sq);
        if (!ctx) return false;

        // Linearize factor
        ICPFactor::linearize(*ctx, factor_setting, H, b, e);
        return true;
      },
      initial_guess);
}

// =============================================================================
// Point-to-Plane ICP
// =============================================================================

/// @brief Align using Point-to-Plane ICP
/// @pre Target must have normals (call geometry::estimateNormals first)
/// @param source Source point cloud
/// @param target Target point cloud (with normals)
/// @param initial_guess Initial transformation estimate
/// @param settings Registration settings
/// @return Registration result
[[nodiscard]] inline RegistrationResult alignPlaneICP(
    const PointCloud& source,
    const PointCloud& target,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const AlignSettings& settings = {}) {
  if (source.empty() || target.empty()) {
    return {initial_guess, 0.0, std::numeric_limits<double>::infinity(), 0, false, std::nullopt};
  }

  if (!target.hasNormal()) {
    throw std::runtime_error(
        "alignPlaneICP: target must have normals. "
        "Call geometry::estimateNormals() first.");
  }

  // Build correspondence finder
  KdTreeCorrespondence correspondence;
  correspondence.build(target);

  const size_t n = source.size();
  const float max_dist_sq =
      settings.max_correspondence_dist * settings.max_correspondence_dist;
  PlaneICPFactor::Setting factor_setting;
  factor_setting.robust_kernel = settings.robust_kernel;
  factor_setting.robust_kernel_width = settings.robust_kernel_width;

  // Configure solver
  IterativeSolver<> solver;
  solver.setMaxIterations(settings.max_iterations);
  solver.setConvergenceThresholds(settings.translation_eps, settings.rotation_eps);
  solver.setMinCorrespondences(settings.min_correspondences);
  solver.criteria().relative_error_eps = settings.relative_error_eps;

  return solver.solve(
      n,
      [&](size_t idx, const Eigen::Isometry3d& T, double* H, double* b, double* e) {
        // Transform source point
        const Eigen::Vector3d p_src = T * source[idx].head<3>().cast<double>();

        // Find correspondence with normal
        auto ctx = correspondence.findWithNormal(p_src, max_dist_sq);
        if (!ctx) return false;

        // Linearize factor
        PlaneICPFactor::linearize(*ctx, factor_setting, H, b, e);
        return true;
      },
      initial_guess);
}

// =============================================================================
// GICP
// =============================================================================

/// @brief Align using Generalized ICP (GICP)
/// @pre Both source and target must have covariances
/// @param source Source point cloud (with covariances)
/// @param target Target point cloud (with covariances)
/// @param initial_guess Initial transformation estimate
/// @param settings Registration settings
/// @return Registration result
[[nodiscard]] inline RegistrationResult alignGICP(
    const PointCloud& source,
    const PointCloud& target,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const AlignSettings& settings = {}) {
  if (source.empty() || target.empty()) {
    return {initial_guess, 0.0, std::numeric_limits<double>::infinity(), 0, false, std::nullopt};
  }

  if (!source.hasCovariance()) {
    throw std::runtime_error(
        "alignGICP: source must have covariances. "
        "Call geometry::estimateCovariances() first.");
  }
  if (!target.hasCovariance()) {
    throw std::runtime_error(
        "alignGICP: target must have covariances. "
        "Call geometry::estimateCovariances() first.");
  }

  // Pre-compute regularized covariances
  auto source_cov_reg =
      precomputeRegularizedCovariances(source, settings.covariance_epsilon);
  auto target_cov_reg =
      precomputeRegularizedCovariances(target, settings.covariance_epsilon);

  // Build correspondence finder
  KdTreeCorrespondence correspondence;
  correspondence.build(target);
  correspondence.setRegularizedCovariances(&target_cov_reg);

  const size_t n = source.size();
  const float max_dist_sq =
      settings.max_correspondence_dist * settings.max_correspondence_dist;
  GICPFactor::Setting factor_setting;
  factor_setting.robust_kernel = settings.robust_kernel;
  factor_setting.robust_kernel_width = settings.robust_kernel_width;

  // Configure solver
  IterativeSolver<> solver;
  solver.setMaxIterations(settings.max_iterations);
  solver.setConvergenceThresholds(settings.translation_eps, settings.rotation_eps);
  solver.setMinCorrespondences(settings.min_correspondences);
  solver.criteria().relative_error_eps = settings.relative_error_eps;

  return solver.solve(
      n,
      [&](size_t idx, const Eigen::Isometry3d& T, double* H, double* b, double* e) {
        // Transform source point
        const Eigen::Vector3d p_src = T * source[idx].head<3>().cast<double>();
        const Eigen::Matrix3d R = T.rotation();

        // Find correspondence with covariance
        auto ctx = correspondence.findWithCovariance(p_src, R, source_cov_reg[idx], max_dist_sq);
        if (!ctx) return false;

        // Linearize factor
        GICPFactor::linearize(*ctx, factor_setting, H, b, e);
        return true;
      },
      initial_guess);
}

// =============================================================================
// Voxelized GICP (VGICP)
// =============================================================================

/// @brief Align using Voxelized GICP with prebuilt voxel map (fastest)
///
/// Use this overload when registering multiple scans against the same target.
/// Build the VoxelCorrespondence once, then call this function repeatedly.
///
/// @pre Source must have covariances (call geometry::estimateCovariances first)
/// @param source Source point cloud (with covariances)
/// @param correspondence Prebuilt voxel correspondence finder
/// @param initial_guess Initial transformation estimate
/// @param settings Registration settings
/// @return Registration result
///
/// Example (SLAM-style usage):
/// @code
///   // Build voxel map once
///   VoxelCorrespondence local_map(0.5f);
///   local_map.build(target);
///
///   // Register multiple scans (fast!)
///   for (auto& scan : scans) {
///     geometry::estimateCovariances(scan, 20);
///     auto result = alignVGICP(scan, local_map);
///   }
/// @endcode
[[nodiscard]] inline RegistrationResult alignVGICP(
    const PointCloud& source,
    const VoxelCorrespondence& correspondence,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const AlignSettings& settings = {}) {
  if (source.empty() || correspondence.empty()) {
    return {initial_guess, 0.0, std::numeric_limits<double>::infinity(), 0, false, std::nullopt};
  }

  if (!source.hasCovariance()) {
    throw std::runtime_error(
        "alignVGICP: source must have covariances. "
        "Call geometry::estimateCovariances() first.");
  }

  // Pre-compute regularized source covariances
  auto source_cov_reg =
      precomputeRegularizedCovariances(source, settings.covariance_epsilon);

  const size_t n = source.size();
  VGICPFactor::Setting factor_setting;
  factor_setting.robust_kernel = settings.robust_kernel;
  factor_setting.robust_kernel_width = settings.robust_kernel_width;

  // Configure solver
  IterativeSolver<> solver;
  solver.setMaxIterations(settings.max_iterations);
  solver.setConvergenceThresholds(settings.translation_eps, settings.rotation_eps);
  solver.setMinCorrespondences(settings.min_correspondences);
  solver.criteria().relative_error_eps = settings.relative_error_eps;

  return solver.solve(
      n,
      [&](size_t idx, const Eigen::Isometry3d& T, double* H, double* b, double* e) {
        // Transform source point
        const Eigen::Vector3d p_src = T * source[idx].head<3>().cast<double>();
        const Eigen::Matrix3d R = T.rotation();

        // Find correspondence via O(1) voxel lookup
        auto ctx = correspondence.find(p_src, R, source_cov_reg[idx]);
        if (!ctx) return false;

        // Linearize factor
        VGICPFactor::linearize(*ctx, factor_setting, H, b, e);
        return true;
      },
      initial_guess);
}

/// @brief Align using Voxelized GICP (convenience function)
///
/// Builds voxel map internally. For repeated registration against the same
/// target, use the overload that takes a prebuilt VoxelCorrespondence.
///
/// @pre Source must have covariances (call geometry::estimateCovariances first)
/// @param source Source point cloud (with covariances)
/// @param target Target point cloud
/// @param voxel_resolution Voxel size in meters
/// @param initial_guess Initial transformation estimate
/// @param settings Registration settings
/// @return Registration result
[[nodiscard]] inline RegistrationResult alignVGICP(
    const PointCloud& source,
    const PointCloud& target,
    float voxel_resolution = 1.0f,
    const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity(),
    const AlignSettings& settings = {}) {
  if (source.empty() || target.empty()) {
    return {initial_guess, 0.0, std::numeric_limits<double>::infinity(), 0, false, std::nullopt};
  }

  // Build voxel correspondence finder
  VoxelCorrespondence correspondence(voxel_resolution, settings.covariance_epsilon);
  correspondence.build(target);

  // Delegate to prebuilt overload
  return alignVGICP(source, correspondence, initial_guess, settings);
}

} // namespace registration
} // namespace nanopcl

#endif // NANOPCL_REGISTRATION_ALIGN_HPP
