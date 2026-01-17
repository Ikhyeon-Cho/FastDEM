// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// GICP (Generalized ICP) optimizer using Mahalanobis distance weighting.
// Do not include this file directly; include <nanopcl/registration/gicp.hpp>
//
// Reference: Segal, Haehnel, Thrun. "Generalized-ICP" (RSS 2009)
//
// Performance optimizations:
//   - Covariance regularization is pre-computed outside the loop
//   - Analytical 3x3 matrix inverse (faster than Eigen::inverse())
//   - OpenMP parallel Hessian accumulation

#ifndef NANOPCL_REGISTRATION_IMPL_GICP_OPTIMIZER_HPP
#define NANOPCL_REGISTRATION_IMPL_GICP_OPTIMIZER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/registration/correspondence.hpp"
#include "nanopcl/registration/robust_kernels.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace npcl {
namespace registration {
namespace detail {

/**
 * @brief Compute skew-symmetric matrix [v]×
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}

/**
 * @brief Analytical 3x3 matrix inverse using Cramer's rule
 *
 * Faster than Eigen::inverse() for small matrices.
 * Assumes matrix is invertible (determinant != 0).
 */
inline Eigen::Matrix3d inverse3x3(const Eigen::Matrix3d& M) {
  double det = M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) -
               M(0, 1) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0)) +
               M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0));

  double inv_det = 1.0 / det;

  Eigen::Matrix3d inv;
  inv(0, 0) = (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) * inv_det;
  inv(0, 1) = (M(0, 2) * M(2, 1) - M(0, 1) * M(2, 2)) * inv_det;
  inv(0, 2) = (M(0, 1) * M(1, 2) - M(0, 2) * M(1, 1)) * inv_det;
  inv(1, 0) = (M(1, 2) * M(2, 0) - M(1, 0) * M(2, 2)) * inv_det;
  inv(1, 1) = (M(0, 0) * M(2, 2) - M(0, 2) * M(2, 0)) * inv_det;
  inv(1, 2) = (M(0, 2) * M(1, 0) - M(0, 0) * M(1, 2)) * inv_det;
  inv(2, 0) = (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0)) * inv_det;
  inv(2, 1) = (M(0, 1) * M(2, 0) - M(0, 0) * M(2, 1)) * inv_det;
  inv(2, 2) = (M(0, 0) * M(1, 1) - M(0, 1) * M(1, 0)) * inv_det;

  return inv;
}

/**
 * @brief Regularize covariance matrix for GICP (single matrix)
 *
 * Following Segal et al. (RSS 2009), normalizes eigenvalues to achieve
 * "plane-to-plane" alignment effect:
 *   - Smallest eigenvalue (plane normal direction) → epsilon
 *   - Other eigenvalues → 1.0 (tangent plane directions)
 *
 * This ensures uniform weighting and fast convergence.
 */
inline Eigen::Matrix3d regularizeCovariance(const Eigen::Matrix3f& cov,
                                            double epsilon) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov.cast<double>());

  // Normalize per Segal et al. (RSS 2009): [epsilon, 1, 1]
  Eigen::Vector3d eigenvalues;
  eigenvalues(0) = epsilon;  // Smallest: plane normal direction
  eigenvalues(1) = 1.0;      // Tangent direction 1
  eigenvalues(2) = 1.0;      // Tangent direction 2

  return solver.eigenvectors() * eigenvalues.asDiagonal() *
         solver.eigenvectors().transpose();
}

/**
 * @brief Pre-compute regularized covariances for all points
 *
 * This should be called ONCE before the GICP iteration loop.
 * Parallelized with OpenMP.
 *
 * @param cloud Point cloud with raw covariances
 * @param epsilon Regularization value
 * @return Vector of regularized covariance matrices (double precision)
 */
inline std::vector<Eigen::Matrix3d> precomputeRegularizedCovariances(
    const PointCloud& cloud, double epsilon) {
  const size_t n = cloud.size();
  std::vector<Eigen::Matrix3d> regularized(n);

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (size_t i = 0; i < n; ++i) {
    const auto& cov = cloud.covariance()[i];
    if (std::isfinite(cov(0, 0))) {
      regularized[i] = regularizeCovariance(cov, epsilon);
    } else {
      // Invalid covariance - use identity (will be skipped anyway)
      regularized[i] = Eigen::Matrix3d::Identity() * epsilon;
    }
  }

  return regularized;
}

/**
 * @brief Solve GICP optimization using Gauss-Newton
 *
 * Optimized implementation using upper-triangular scalar accumulation.
 * Exploits block structure of H = J^T W J where J = [I, -[Tp]×]:
 *   H_11 = W (symmetric 3x3)
 *   H_12 = -W * [Tp]× (3x3)
 *   H_22 = [Tp]× * W * [Tp]× (symmetric 3x3)
 *
 * @param source Source point cloud
 * @param target Target point cloud
 * @param source_cov_reg Pre-regularized source covariances
 * @param target_cov_reg Pre-regularized target covariances
 * @param T_current Current transformation estimate
 * @param correspondences Valid correspondences
 * @param robust_kernel Robust kernel type for M-estimation
 * @param robust_kernel_width Kernel width parameter (k)
 * @param hessian_out Optional output for Hessian matrix (for covariance
 * estimation)
 * @return Incremental transformation delta_T
 */
inline Eigen::Isometry3d solveGICP(
    const PointCloud& source, const PointCloud& target,
    const std::vector<Eigen::Matrix3d>& source_cov_reg,
    const std::vector<Eigen::Matrix3d>& target_cov_reg,
    const Eigen::Isometry3d& T_current, const Correspondences& correspondences,
    RobustKernel robust_kernel = RobustKernel::NONE,
    double robust_kernel_width = 1.0,
    Eigen::Matrix<double, 6, 6>* hessian_out = nullptr) {
  // H is symmetric 6x6, we only need upper triangular (21 elements)
  // Layout: [H00, H01, H02, H03, H04, H05, H11, H12, H13, H14, H15,
  //          H22, H23, H24, H25, H33, H34, H35, H44, H45, H55]
  double H_global[21] = {0};
  double b_global[6] = {0};

  const Eigen::Matrix3d R = T_current.rotation();

#ifdef _OPENMP
  const int num_threads = omp_get_max_threads();

  // Thread-local buffers: 21 (H) + 6 (b) = 27 doubles = 216 bytes per thread
  std::vector<std::array<double, 27>> locals(num_threads);
  for (auto& arr : locals) arr.fill(0.0);

#pragma omp parallel
  {
    const int tid = omp_get_thread_num();
    double* H_local = locals[tid].data();       // [0..20]
    double* b_local = locals[tid].data() + 21;  // [21..26]

#pragma omp for nowait
    for (size_t i = 0; i < correspondences.size(); ++i) {
      const auto& corr = correspondences[i];

      // Load points
      Eigen::Vector3d p = source[corr.source_idx].cast<double>();
      Eigen::Vector3d q = target[corr.target_idx].cast<double>();

      // Transform source point
      Eigen::Vector3d Tp = T_current * p;
      double tx = Tp.x(), ty = Tp.y(), tz = Tp.z();

      // Residual: r = Tp - q
      double rx = Tp.x() - q.x();
      double ry = Tp.y() - q.y();
      double rz = Tp.z() - q.z();

      // Combined covariance: C = C_B + R * C_A * R^T
      const Eigen::Matrix3d& C_A = source_cov_reg[corr.source_idx];
      const Eigen::Matrix3d& C_B = target_cov_reg[corr.target_idx];
      Eigen::Matrix3d C = C_B + R * C_A * R.transpose();

      // Weight matrix: W = C^{-1} (analytical inverse)
      Eigen::Matrix3d W = inverse3x3(C);

      // Extract W elements (symmetric)
      double w00 = W(0, 0), w01 = W(0, 1), w02 = W(0, 2);
      double w11 = W(1, 1), w12 = W(1, 2), w22 = W(2, 2);

      // ================================================================
      // Compute Wr for b = J^T W r = [Wr; [Tp]× * Wr]
      // ================================================================
      double wr0 = w00 * rx + w01 * ry + w02 * rz;
      double wr1 = w01 * rx + w11 * ry + w12 * rz;
      double wr2 = w02 * rx + w12 * ry + w22 * rz;

      // ================================================================
      // Robust weight using Mahalanobis distance (reusing wr values)
      // mahal_sq = r^T W r = r dot (W*r)
      // ================================================================
      double mahal_sq = rx * wr0 + ry * wr1 + rz * wr2;
      double mahal_dist = std::sqrt(std::max(mahal_sq, 0.0));
      double rw =
          computeRobustWeight(mahal_dist, robust_kernel, robust_kernel_width);

      // Scale W elements and wr by robust weight (avoids recomputation)
      w00 *= rw;
      w01 *= rw;
      w02 *= rw;
      w11 *= rw;
      w12 *= rw;
      w22 *= rw;
      wr0 *= rw;
      wr1 *= rw;
      wr2 *= rw;

      // [Tp]× * Wr = Tp.cross(Wr)
      double b3 = ty * wr2 - tz * wr1;
      double b4 = tz * wr0 - tx * wr2;
      double b5 = tx * wr1 - ty * wr0;

      // ================================================================
      // Compute H = J^T W J using block structure (W already scaled)
      // H_11 = W, H_12 = -W*[Tp]×, H_22 = [Tp]×*W*[Tp]×
      // ================================================================

      // H_12 = -W * [Tp]× (compute all 9 elements)
      // [Tp]× columns: [0,tz,-ty], [-tz,0,tx], [ty,-tx,0]
      double h03 = -w01 * tz + w02 * ty;
      double h04 = w00 * tz - w02 * tx;
      double h05 = -w00 * ty + w01 * tx;
      double h13 = -w11 * tz + w12 * ty;
      double h14 = w01 * tz - w12 * tx;
      double h15 = -w01 * ty + w11 * tx;
      double h23 = -w12 * tz + w22 * ty;
      double h24 = w02 * tz - w22 * tx;
      double h25 = -w02 * ty + w12 * tx;

      // H_22 = -[Tp]× * W * [Tp]× (note the negative sign!)
      // Using M = W * [Tp]× = -H_12, we compute [Tp]× * M then negate
      // [Tp]× rows: [0,-tz,ty], [tz,0,-tx], [-ty,tx,0]
      // H_22[i,j] = -([Tp]×_row_i dot M_col_j)
      double h33 = -(tz * h13 - ty * h23);
      double h34 = -(tz * h14 - ty * h24);
      double h35 = -(tz * h15 - ty * h25);
      double h44 = -(-tz * h04 + tx * h24);
      double h45 = -(-tz * h05 + tx * h25);
      double h55 = -(ty * h05 - tx * h15);

      // ================================================================
      // Accumulate upper-triangular H and b
      // ================================================================
      // H_11 block (W, symmetric)
      H_local[0] += w00;
      H_local[1] += w01;
      H_local[2] += w02;
      H_local[6] += w11;
      H_local[7] += w12;
      H_local[11] += w22;

      // H_12 block (full 3x3)
      H_local[3] += h03;
      H_local[4] += h04;
      H_local[5] += h05;
      H_local[8] += h13;
      H_local[9] += h14;
      H_local[10] += h15;
      H_local[12] += h23;
      H_local[13] += h24;
      H_local[14] += h25;

      // H_22 block (symmetric)
      H_local[15] += h33;
      H_local[16] += h34;
      H_local[17] += h35;
      H_local[18] += h44;
      H_local[19] += h45;
      H_local[20] += h55;

      // b vector
      b_local[0] += wr0;
      b_local[1] += wr1;
      b_local[2] += wr2;
      b_local[3] += b3;
      b_local[4] += b4;
      b_local[5] += b5;
    }
  }

  // Sequential reduction (NO critical section)
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
    Eigen::Vector3d q = target[corr.target_idx].cast<double>();
    Eigen::Vector3d Tp = T_current * p;
    double tx = Tp.x(), ty = Tp.y(), tz = Tp.z();

    double rx = Tp.x() - q.x();
    double ry = Tp.y() - q.y();
    double rz = Tp.z() - q.z();

    const Eigen::Matrix3d& C_A = source_cov_reg[corr.source_idx];
    const Eigen::Matrix3d& C_B = target_cov_reg[corr.target_idx];
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

    // H_12 = -W * [Tp]×
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

  // ================================================================
  // Reconstruct symmetric 6x6 matrix from upper triangular
  // ================================================================
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Matrix<double, 6, 1> b;

  int idx = 0;
  for (int row = 0; row < 6; ++row) {
    for (int col = row; col < 6; ++col) {
      H(row, col) = H_global[idx];
      H(col, row) = H_global[idx];  // Symmetric fill
      idx++;
    }
    b(row) = b_global[row];
  }

  // Output Hessian if requested (for covariance estimation)
  if (hessian_out) {
    *hessian_out = H;
  }

  // ================================================================
  // Solve linear system: H * dx = -b
  // ================================================================
  Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-b);

  // Convert twist [t; omega] to Isometry3d
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
}  // namespace registration
}  // namespace npcl

#endif  // NANOPCL_REGISTRATION_IMPL_GICP_OPTIMIZER_HPP
