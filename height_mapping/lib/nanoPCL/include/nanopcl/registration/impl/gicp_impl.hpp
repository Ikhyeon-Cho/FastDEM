// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// GICP implementation.
// Do not include this file directly; include <nanopcl/registration/gicp.hpp>

#ifndef NANOPCL_REGISTRATION_IMPL_GICP_IMPL_HPP
#define NANOPCL_REGISTRATION_IMPL_GICP_IMPL_HPP

#include <cmath>
#include <limits>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "nanopcl/registration/impl/gicp_optimizer.hpp"

namespace npcl {
namespace registration {

template <typename SearchMethod>
RegistrationResult gicp(const PointCloud& source, const PointCloud& target,
                        const SearchMethod& target_search,
                        const Eigen::Isometry3d& initial_guess,
                        const GICPConfig& config) {
  // ==========================================================================
  // Preconditions: both clouds must have covariances
  // ==========================================================================
  if (!source.hasCovariance()) {
    throw std::runtime_error(
        "gicp: Source cloud missing covariances. "
        "Call geometry::estimateCovariances() first.");
  }
  if (!target.hasCovariance()) {
    throw std::runtime_error(
        "gicp: Target cloud missing covariances. "
        "Call geometry::estimateCovariances() first.");
  }

  const size_t n = source.size();

  // Handle edge cases
  if (n == 0) {
    return {initial_guess, 0.0, std::numeric_limits<double>::infinity(), 0,
            false};
  }

  // ==========================================================================
  // Pre-compute regularized covariances (CRITICAL OPTIMIZATION)
  // This is done ONCE before the iteration loop, not per-correspondence.
  // ==========================================================================
  std::vector<Eigen::Matrix3d> source_cov_reg =
      detail::precomputeRegularizedCovariances(source,
                                               config.covariance_epsilon);
  std::vector<Eigen::Matrix3d> target_cov_reg =
      detail::precomputeRegularizedCovariances(target,
                                               config.covariance_epsilon);

  // ==========================================================================
  // Pre-allocate buffer (reused across iterations)
  // ==========================================================================
  Correspondences correspondences;
  correspondences.resize(n);  // Pre-allocate for parallel write

  Eigen::Isometry3d T_current = initial_guess;
  double prev_mse = std::numeric_limits<double>::max();
  Matrix6d last_hessian;  // For covariance estimation

  for (int iter = 0; iter < config.max_iterations; ++iter) {
    // ========================================================================
    // Step 1-2: Transform source points and find correspondences (PARALLEL)
    // ========================================================================
    correspondences.resize(n);  // Restore full size for parallel write
    double sum_dist_sq = 0.0;

#ifdef _OPENMP
#pragma omp parallel for reduction(+ : sum_dist_sq)
#endif
    for (size_t i = 0; i < n; ++i) {
      // Transform source point with current estimate
      Point src_transformed =
          (T_current * source[i].cast<double>()).cast<float>();

      // Find nearest point in target
      auto nearest = target_search.nearest(src_transformed,
                                           config.max_correspondence_dist);

      if (!nearest) {
        // Mark as invalid
        correspondences[i].source_idx = INVALID_INDEX;
        continue;
      }

      // Skip if source covariance is invalid (NaN check on original)
      const auto& src_cov = source.covariance()[i];
      if (!std::isfinite(src_cov(0, 0))) {
        correspondences[i].source_idx = INVALID_INDEX;
        continue;
      }

      // Skip if target covariance is invalid (NaN check on original)
      const auto& tgt_cov = target.covariance()[nearest->index];
      if (!std::isfinite(tgt_cov(0, 0))) {
        correspondences[i].source_idx = INVALID_INDEX;
        continue;
      }

      correspondences[i] = {static_cast<uint32_t>(i), nearest->index,
                            nearest->dist_sq};
      sum_dist_sq += nearest->dist_sq;
    }

    // ========================================================================
    // Compact correspondences (remove invalids) - O(N) single pass
    // ========================================================================
    size_t valid_count = 0;
    for (size_t i = 0; i < n; ++i) {
      if (correspondences[i].source_idx != INVALID_INDEX) {
        if (valid_count != i) {
          correspondences[valid_count] = correspondences[i];
        }
        ++valid_count;
      }
    }
    correspondences.resize(valid_count);

    // ========================================================================
    // Validity check
    // ========================================================================
    if (correspondences.size() < config.min_correspondences) {
      return {T_current, 0.0, std::numeric_limits<double>::infinity(),
              static_cast<size_t>(iter), false};
    }

    double mse = sum_dist_sq / static_cast<double>(correspondences.size());

    // ========================================================================
    // Step 3: GICP optimization (using pre-computed regularized covariances)
    // ========================================================================
    Eigen::Isometry3d delta_T =
        detail::solveGICP(source, target, source_cov_reg, target_cov_reg,
                          T_current, correspondences, config.robust_kernel,
                          config.robust_kernel_width, &last_hessian);

    // ========================================================================
    // Step 4: Update cumulative transform
    // ========================================================================
    T_current = delta_T * T_current;

    // ========================================================================
    // Step 5: Check convergence
    // ========================================================================

    // Check 1: Delta transform convergence (transformation is stable)
    double t_norm = delta_T.translation().norm();
    double r_angle = std::abs(Eigen::AngleAxisd(delta_T.rotation()).angle());
    bool is_stable = (t_norm < config.translation_threshold) &&
                     (r_angle < config.rotation_threshold);

    // Check 2: Relative MSE convergence (no further improvement)
    double relative_mse_change = std::abs(prev_mse - mse) / prev_mse;
    bool is_stalled = (relative_mse_change < config.relative_mse_threshold);
    prev_mse = mse;

    // Converged if stable OR stalled
    if (is_stable || is_stalled) {
      double fitness =
          static_cast<double>(correspondences.size()) / static_cast<double>(n);
      double rmse = std::sqrt(mse);

      // Compute covariance from Hessian inverse
      auto ldlt = last_hessian.ldlt();
      std::optional<Matrix6d> cov;
      if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
        cov = ldlt.solve(Matrix6d::Identity());
      }

      return {T_current, fitness, rmse, static_cast<size_t>(iter + 1),
              true,      cov};
    }
  }

  // Max iterations reached without convergence
  double fitness =
      static_cast<double>(correspondences.size()) / static_cast<double>(n);
  double rmse = std::sqrt(prev_mse);

  // Compute covariance even if not converged (may still be useful)
  auto ldlt = last_hessian.ldlt();
  std::optional<Matrix6d> cov;
  if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
    cov = ldlt.solve(Matrix6d::Identity());
  }

  return {T_current, fitness, rmse, static_cast<size_t>(config.max_iterations),
          false,     cov};
}

template <typename SearchMethod>
RegistrationResult gicp(const PointCloud& source, const PointCloud& target,
                        const Eigen::Isometry3d& initial_guess,
                        const GICPConfig& config) {
  // Early exit checks
  if (!source.hasCovariance()) {
    throw std::runtime_error(
        "gicp: Source cloud missing covariances. "
        "Call geometry::estimateCovariances() first.");
  }
  if (!target.hasCovariance()) {
    throw std::runtime_error(
        "gicp: Target cloud missing covariances. "
        "Call geometry::estimateCovariances() first.");
  }

  SearchMethod searcher;
  searcher.build(target);
  return gicp(source, target, searcher, initial_guess, config);
}

}  // namespace registration
}  // namespace npcl

#endif  // NANOPCL_REGISTRATION_IMPL_GICP_IMPL_HPP
