// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Optimized Gauss-Newton optimizer for Point-to-Plane ICP.
// Do not include this file directly; include
// <nanopcl/registration/icp_plane.hpp>
//
// Optimization techniques:
//   1. Upper triangular accumulation (21 elements instead of 36)
//   2. Scalar operations for SIMD auto-vectorization
//   3. Lock-free parallel reduction using thread-local arrays
//   4. Stack-based buffers for L1 cache locality

#ifndef NANOPCL_REGISTRATION_IMPL_GAUSS_NEWTON_OPTIMIZER_HPP
#define NANOPCL_REGISTRATION_IMPL_GAUSS_NEWTON_OPTIMIZER_HPP

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
 * @brief Solve Point-to-Plane optimization using Gauss-Newton
 *
 * Minimizes: E = sum_i w_i * ((T * s_i - t_i) . n_i)^2
 *
 * Uses linearization with small angle approximation:
 *   delta_T ≈ I + [omega]x + t
 *
 * Jacobian per point: J = [n^T, (s × n)^T] (1×6)
 * Normal equation: (J^T W J) * dx = -J^T W r
 *
 * @param source Source point cloud
 * @param target Target point cloud (with normals)
 * @param T_current Current transformation estimate
 * @param correspondences Valid correspondences
 * @param robust_kernel Robust kernel type for M-estimation
 * @param robust_kernel_width Kernel width parameter (k)
 * @param hessian_out Optional output for Hessian matrix (for covariance
 * estimation)
 * @return Incremental transformation delta_T
 */
inline Eigen::Isometry3d solvePointToPlane(
    const PointCloud& source, const PointCloud& target,
    const Eigen::Isometry3d& T_current, const Correspondences& correspondences,
    RobustKernel robust_kernel = RobustKernel::NONE,
    double robust_kernel_width = 1.0,
    Eigen::Matrix<double, 6, 6>* hessian_out = nullptr) {
  // H is symmetric 6x6, we only need upper triangular (21 elements)
  // Layout: [H00, H01, H02, H03, H04, H05, H11, H12, ..., H55]
  // idx(r,c) = r*6 - r*(r+1)/2 + c  for c >= r
  double H_global[21] = {0};
  double b_global[6] = {0};

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

      // Load and transform source point
      Eigen::Vector3d s = T_current * source[corr.source_idx].cast<double>();
      Eigen::Vector3d q = target[corr.target_idx].cast<double>();
      Eigen::Vector3d n = target.normal()[corr.target_idx].cast<double>();

      // Residual: r = (s - q) . n
      double residual = (s - q).dot(n);

      // Robust weight (M-estimator)
      double w =
          computeRobustWeight(residual, robust_kernel, robust_kernel_width);

      // Jacobian: J = [nx, ny, nz, (s×n)_x, (s×n)_y, (s×n)_z]
      double J[6] = {
          n.x(),
          n.y(),
          n.z(),
          s.y() * n.z() - s.z() * n.y(),  // (s × n)_x
          s.z() * n.x() - s.x() * n.z(),  // (s × n)_y
          s.x() * n.y() - s.y() * n.x()   // (s × n)_z
      };

      // Accumulate upper triangular H and b (with robust weight)
      int idx = 0;
      for (int row = 0; row < 6; ++row) {
        for (int col = row; col < 6; ++col) {
          H_local[idx++] += w * J[row] * J[col];
        }
        b_local[row] += w * J[row] * residual;
      }
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

    Eigen::Vector3d s = T_current * source[corr.source_idx].cast<double>();
    Eigen::Vector3d q = target[corr.target_idx].cast<double>();
    Eigen::Vector3d n = target.normal()[corr.target_idx].cast<double>();

    double residual = (s - q).dot(n);

    // Robust weight (M-estimator)
    double w =
        computeRobustWeight(residual, robust_kernel, robust_kernel_width);

    double J[6] = {n.x(),
                   n.y(),
                   n.z(),
                   s.y() * n.z() - s.z() * n.y(),
                   s.z() * n.x() - s.x() * n.z(),
                   s.x() * n.y() - s.y() * n.x()};

    int idx = 0;
    for (int row = 0; row < 6; ++row) {
      for (int col = row; col < 6; ++col) {
        H_global[idx++] += w * J[row] * J[col];
      }
      b_global[row] += w * J[row] * residual;
    }
  }
#endif

  // ==========================================================================
  // Reconstruct symmetric 6x6 matrix from upper triangular
  // ==========================================================================
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

  // ==========================================================================
  // Solve linear system: H * dx = -b
  // LDLT is robust and efficient for symmetric positive semi-definite matrices
  // ==========================================================================
  Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-b);

  // ==========================================================================
  // Convert twist [t; omega] to Isometry3d
  // ==========================================================================
  Eigen::Vector3d t = dx.head<3>();
  Eigen::Vector3d omega = dx.tail<3>();

  Eigen::Isometry3d delta_T = Eigen::Isometry3d::Identity();
  delta_T.translation() = t;

  // Rotation from angle-axis (exact, not small angle approximation)
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

#endif  // NANOPCL_REGISTRATION_IMPL_GAUSS_NEWTON_OPTIMIZER_HPP
