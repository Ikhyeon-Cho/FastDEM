// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// VGICP implementation.
// Do not include this file directly; include <nanopcl/registration/vgicp.hpp>

#ifndef NANOPCL_REGISTRATION_IMPL_VGICP_IMPL_HPP
#define NANOPCL_REGISTRATION_IMPL_VGICP_IMPL_HPP

#include <cmath>
#include <limits>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "nanopcl/registration/impl/gicp_optimizer.hpp"

namespace npcl {
namespace registration {

namespace detail {

/**
 * @brief Pre-compute regularized covariances for source cloud
 */
inline std::vector<Eigen::Matrix3d> precomputeSourceCovariances(
    const PointCloud& source, double epsilon) {
  const size_t n = source.size();
  std::vector<Eigen::Matrix3d> regularized(n);

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (size_t i = 0; i < n; ++i) {
    const auto& cov = source.covariance()[i];
    if (std::isfinite(cov(0, 0))) {
      // Regularize per Segal et al. (RSS 2009)
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov.cast<double>());
      Eigen::Vector3d eigenvalues;
      eigenvalues(0) = epsilon;
      eigenvalues(1) = 1.0;
      eigenvalues(2) = 1.0;
      regularized[i] = solver.eigenvectors() * eigenvalues.asDiagonal() *
                       solver.eigenvectors().transpose();
    } else {
      regularized[i] = Eigen::Matrix3d::Identity() * epsilon;
    }
  }

  return regularized;
}

/**
 * @brief VGICP correspondence with voxel distribution
 */
struct VGICPCorrespondence {
  uint32_t source_idx;
  Eigen::Vector3f target_mean;     ///< Voxel centroid (replaces target point)
  Eigen::Matrix3f target_cov_reg;  ///< Regularized voxel covariance
  float dist_sq;
};

/**
 * @brief Solve VGICP optimization using Gauss-Newton
 *
 * Same as GICP solver but uses voxel mean instead of target point.
 */
inline Eigen::Isometry3d solveVGICP(
    const PointCloud& source,
    const std::vector<Eigen::Matrix3d>& source_cov_reg,
    const std::vector<VGICPCorrespondence>& correspondences,
    const Eigen::Isometry3d& T_current,
    RobustKernel robust_kernel = RobustKernel::NONE,
    double robust_kernel_width = 1.0,
    Eigen::Matrix<double, 6, 6>* hessian_out = nullptr) {
  double H_global[21] = {0};
  double b_global[6] = {0};

  const Eigen::Matrix3d R = T_current.rotation();

#ifdef _OPENMP
  const int num_threads = omp_get_max_threads();
  std::vector<std::array<double, 27>> locals(num_threads);
  for (auto& arr : locals) arr.fill(0.0);

#pragma omp parallel
  {
    const int tid = omp_get_thread_num();
    double* H_local = locals[tid].data();
    double* b_local = locals[tid].data() + 21;

#pragma omp for nowait
    for (size_t i = 0; i < correspondences.size(); ++i) {
      const auto& corr = correspondences[i];

      // Source point and voxel mean (target)
      Eigen::Vector3d p = source[corr.source_idx].cast<double>();
      Eigen::Vector3d q = corr.target_mean.cast<double>();

      // Transform source point
      Eigen::Vector3d Tp = T_current * p;
      double tx = Tp.x(), ty = Tp.y(), tz = Tp.z();

      // Residual: r = Tp - q (voxel mean)
      double rx = Tp.x() - q.x();
      double ry = Tp.y() - q.y();
      double rz = Tp.z() - q.z();

      // Combined covariance: C = C_B + R * C_A * R^T
      const Eigen::Matrix3d& C_A = source_cov_reg[corr.source_idx];
      Eigen::Matrix3d C_B = corr.target_cov_reg.cast<double>();
      Eigen::Matrix3d C = C_B + R * C_A * R.transpose();

      // Weight matrix: W = C^{-1}
      Eigen::Matrix3d W = inverse3x3(C);

      double w00 = W(0, 0), w01 = W(0, 1), w02 = W(0, 2);
      double w11 = W(1, 1), w12 = W(1, 2), w22 = W(2, 2);

      // Compute Wr
      double wr0 = w00 * rx + w01 * ry + w02 * rz;
      double wr1 = w01 * rx + w11 * ry + w12 * rz;
      double wr2 = w02 * rx + w12 * ry + w22 * rz;

      // Robust weight using Mahalanobis distance (reusing wr values)
      double mahal_sq = rx * wr0 + ry * wr1 + rz * wr2;
      double mahal_dist = std::sqrt(std::max(mahal_sq, 0.0));
      double rw =
          computeRobustWeight(mahal_dist, robust_kernel, robust_kernel_width);

      // Scale W elements and wr by robust weight
      w00 *= rw;
      w01 *= rw;
      w02 *= rw;
      w11 *= rw;
      w12 *= rw;
      w22 *= rw;
      wr0 *= rw;
      wr1 *= rw;
      wr2 *= rw;

      // b = [Wr; [Tp]× * Wr]
      double b3 = ty * wr2 - tz * wr1;
      double b4 = tz * wr0 - tx * wr2;
      double b5 = tx * wr1 - ty * wr0;

      // H_12 = -W * [Tp]× (W already scaled)
      double h03 = -w01 * tz + w02 * ty;
      double h04 = w00 * tz - w02 * tx;
      double h05 = -w00 * ty + w01 * tx;
      double h13 = -w11 * tz + w12 * ty;
      double h14 = w01 * tz - w12 * tx;
      double h15 = -w01 * ty + w11 * tx;
      double h23 = -w12 * tz + w22 * ty;
      double h24 = w02 * tz - w22 * tx;
      double h25 = -w02 * ty + w12 * tx;

      // H_22 = -[Tp]× * W * [Tp]×
      double h33 = -(tz * h13 - ty * h23);
      double h34 = -(tz * h14 - ty * h24);
      double h35 = -(tz * h15 - ty * h25);
      double h44 = -(-tz * h04 + tx * h24);
      double h45 = -(-tz * h05 + tx * h25);
      double h55 = -(ty * h05 - tx * h15);

      // Accumulate
      H_local[0] += w00;
      H_local[1] += w01;
      H_local[2] += w02;
      H_local[6] += w11;
      H_local[7] += w12;
      H_local[11] += w22;

      H_local[3] += h03;
      H_local[4] += h04;
      H_local[5] += h05;
      H_local[8] += h13;
      H_local[9] += h14;
      H_local[10] += h15;
      H_local[12] += h23;
      H_local[13] += h24;
      H_local[14] += h25;

      H_local[15] += h33;
      H_local[16] += h34;
      H_local[17] += h35;
      H_local[18] += h44;
      H_local[19] += h45;
      H_local[20] += h55;

      b_local[0] += wr0;
      b_local[1] += wr1;
      b_local[2] += wr2;
      b_local[3] += b3;
      b_local[4] += b4;
      b_local[5] += b5;
    }
  }

  // Sequential reduction
  for (int t = 0; t < num_threads; ++t) {
    const double* H_local = locals[t].data();
    const double* b_local = locals[t].data() + 21;
    for (int k = 0; k < 21; ++k) H_global[k] += H_local[k];
    for (int k = 0; k < 6; ++k) b_global[k] += b_local[k];
  }

#else
  // Sequential version
  for (size_t i = 0; i < correspondences.size(); ++i) {
    const auto& corr = correspondences[i];

    Eigen::Vector3d p = source[corr.source_idx].cast<double>();
    Eigen::Vector3d q = corr.target_mean.cast<double>();
    Eigen::Vector3d Tp = T_current * p;
    double tx = Tp.x(), ty = Tp.y(), tz = Tp.z();

    double rx = Tp.x() - q.x();
    double ry = Tp.y() - q.y();
    double rz = Tp.z() - q.z();

    const Eigen::Matrix3d& C_A = source_cov_reg[corr.source_idx];
    Eigen::Matrix3d C_B = corr.target_cov_reg.cast<double>();
    Eigen::Matrix3d C = C_B + R * C_A * R.transpose();
    Eigen::Matrix3d W = inverse3x3(C);

    double w00 = W(0, 0), w01 = W(0, 1), w02 = W(0, 2);
    double w11 = W(1, 1), w12 = W(1, 2), w22 = W(2, 2);

    // Compute Wr
    double wr0 = w00 * rx + w01 * ry + w02 * rz;
    double wr1 = w01 * rx + w11 * ry + w12 * rz;
    double wr2 = w02 * rx + w12 * ry + w22 * rz;

    // Robust weight using Mahalanobis distance
    double mahal_sq = rx * wr0 + ry * wr1 + rz * wr2;
    double mahal_dist = std::sqrt(std::max(mahal_sq, 0.0));
    double rw =
        computeRobustWeight(mahal_dist, robust_kernel, robust_kernel_width);

    // Scale W elements and wr by robust weight
    w00 *= rw;
    w01 *= rw;
    w02 *= rw;
    w11 *= rw;
    w12 *= rw;
    w22 *= rw;
    wr0 *= rw;
    wr1 *= rw;
    wr2 *= rw;

    // b = [Wr; Tp × Wr]
    double b3 = ty * wr2 - tz * wr1;
    double b4 = tz * wr0 - tx * wr2;
    double b5 = tx * wr1 - ty * wr0;

    double h03 = -w01 * tz + w02 * ty;
    double h04 = w00 * tz - w02 * tx;
    double h05 = -w00 * ty + w01 * tx;
    double h13 = -w11 * tz + w12 * ty;
    double h14 = w01 * tz - w12 * tx;
    double h15 = -w01 * ty + w11 * tx;
    double h23 = -w12 * tz + w22 * ty;
    double h24 = w02 * tz - w22 * tx;
    double h25 = -w02 * ty + w12 * tx;

    double h33 = -(tz * h13 - ty * h23);
    double h34 = -(tz * h14 - ty * h24);
    double h35 = -(tz * h15 - ty * h25);
    double h44 = -(-tz * h04 + tx * h24);
    double h45 = -(-tz * h05 + tx * h25);
    double h55 = -(ty * h05 - tx * h15);

    H_global[0] += w00;
    H_global[1] += w01;
    H_global[2] += w02;
    H_global[6] += w11;
    H_global[7] += w12;
    H_global[11] += w22;

    H_global[3] += h03;
    H_global[4] += h04;
    H_global[5] += h05;
    H_global[8] += h13;
    H_global[9] += h14;
    H_global[10] += h15;
    H_global[12] += h23;
    H_global[13] += h24;
    H_global[14] += h25;

    H_global[15] += h33;
    H_global[16] += h34;
    H_global[17] += h35;
    H_global[18] += h44;
    H_global[19] += h45;
    H_global[20] += h55;

    b_global[0] += wr0;
    b_global[1] += wr1;
    b_global[2] += wr2;
    b_global[3] += b3;
    b_global[4] += b4;
    b_global[5] += b5;
  }
#endif

  // Reconstruct symmetric 6x6 matrix
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Matrix<double, 6, 1> b;

  int idx = 0;
  for (int row = 0; row < 6; ++row) {
    for (int col = row; col < 6; ++col) {
      H(row, col) = H_global[idx];
      H(col, row) = H_global[idx];
      idx++;
    }
    b(row) = b_global[row];
  }

  // Output Hessian if requested (for covariance estimation)
  if (hessian_out) {
    *hessian_out = H;
  }

  // Solve H * dx = -b
  Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-b);

  // Convert twist to Isometry
  Eigen::Vector3d t = dx.head<3>();
  Eigen::Vector3d omega = dx.tail<3>();

  Eigen::Isometry3d delta_T = Eigen::Isometry3d::Identity();
  delta_T.translation() = t;

  double theta = omega.norm();
  if (theta > 1e-10) {
    delta_T.linear() =
        Eigen::AngleAxisd(theta, omega / theta).toRotationMatrix();
  }

  return delta_T;
}

}  // namespace detail

// =============================================================================
// VGICP with pre-built voxel map
// =============================================================================

inline RegistrationResult vgicp(const PointCloud& source,
                                const VoxelDistributionMap& voxel_map,
                                const Eigen::Isometry3d& initial_guess,
                                const VGICPConfig& config) {
  // Precondition: source must have covariances
  if (!source.hasCovariance()) {
    throw std::runtime_error(
        "vgicp: Source cloud missing covariances. "
        "Call geometry::estimateCovariances() first.");
  }

  if (voxel_map.empty()) {
    throw std::runtime_error("vgicp: Voxel map is empty.");
  }

  const size_t n = source.size();

  if (n == 0) {
    return {initial_guess, 0.0, std::numeric_limits<double>::infinity(), 0,
            false};
  }

  // ===========================================================================
  // Pre-compute regularized source covariances
  // ===========================================================================
  std::vector<Eigen::Matrix3d> source_cov_reg =
      detail::precomputeSourceCovariances(source, config.covariance_epsilon);

  // ===========================================================================
  // Pre-allocate correspondence buffer
  // ===========================================================================
  std::vector<detail::VGICPCorrespondence> correspondences;
  correspondences.resize(n);

  Eigen::Isometry3d T_current = initial_guess;
  double prev_mse = std::numeric_limits<double>::max();
  Matrix6d last_hessian;  // For covariance estimation

  for (int iter = 0; iter < config.max_iterations; ++iter) {
    // =========================================================================
    // Step 1-2: Transform source points and find voxel correspondences
    // =========================================================================
    correspondences.resize(n);
    double sum_dist_sq = 0.0;

#ifdef _OPENMP
#pragma omp parallel for reduction(+ : sum_dist_sq)
#endif
    for (size_t i = 0; i < n; ++i) {
      // Transform source point
      Point src_transformed =
          (T_current * source[i].cast<double>()).cast<float>();

      // O(1) voxel lookup (key difference from GICP!)
      auto voxel_dist = voxel_map.lookupRegularized(src_transformed);

      if (!voxel_dist || !voxel_dist->isValid()) {
        correspondences[i].source_idx = INVALID_INDEX;
        continue;
      }

      // Skip if source covariance is invalid
      const auto& src_cov = source.covariance()[i];
      if (!std::isfinite(src_cov(0, 0))) {
        correspondences[i].source_idx = INVALID_INDEX;
        continue;
      }

      // Compute distance to voxel mean
      float dist_sq = (src_transformed - voxel_dist->mean).squaredNorm();

      correspondences[i].source_idx = static_cast<uint32_t>(i);
      correspondences[i].target_mean = voxel_dist->mean;
      correspondences[i].target_cov_reg = voxel_dist->covariance;
      correspondences[i].dist_sq = dist_sq;

      sum_dist_sq += dist_sq;
    }

    // =========================================================================
    // Compact correspondences - O(N) single pass
    // =========================================================================
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

    // =========================================================================
    // Validity check
    // =========================================================================
    if (correspondences.size() < config.min_correspondences) {
      return {T_current, 0.0, std::numeric_limits<double>::infinity(),
              static_cast<size_t>(iter), false};
    }

    double mse = sum_dist_sq / static_cast<double>(correspondences.size());

    // =========================================================================
    // Step 3: VGICP optimization
    // =========================================================================
    Eigen::Isometry3d delta_T = detail::solveVGICP(
        source, source_cov_reg, correspondences, T_current,
        config.robust_kernel, config.robust_kernel_width, &last_hessian);

    // =========================================================================
    // Step 4: Update cumulative transform
    // =========================================================================
    T_current = delta_T * T_current;

    // =========================================================================
    // Step 5: Check convergence
    // =========================================================================
    double t_norm = delta_T.translation().norm();
    double r_angle = std::abs(Eigen::AngleAxisd(delta_T.rotation()).angle());
    bool is_stable = (t_norm < config.translation_threshold) &&
                     (r_angle < config.rotation_threshold);

    double relative_mse_change = std::abs(prev_mse - mse) / prev_mse;
    bool is_stalled = (relative_mse_change < config.relative_mse_threshold);
    prev_mse = mse;

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

  // Max iterations reached
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

// =============================================================================
// VGICP with internal voxel map construction
// =============================================================================

inline RegistrationResult vgicp(const PointCloud& source,
                                const PointCloud& target,
                                float voxel_resolution,
                                const Eigen::Isometry3d& initial_guess,
                                const VGICPConfig& config) {
  if (!source.hasCovariance()) {
    throw std::runtime_error(
        "vgicp: Source cloud missing covariances. "
        "Call geometry::estimateCovariances() first.");
  }

  VoxelDistributionMap voxel_map(voxel_resolution, config.covariance_epsilon);
  voxel_map.build(target);

  return vgicp(source, voxel_map, initial_guess, config);
}

}  // namespace registration
}  // namespace npcl

#endif  // NANOPCL_REGISTRATION_IMPL_VGICP_IMPL_HPP
