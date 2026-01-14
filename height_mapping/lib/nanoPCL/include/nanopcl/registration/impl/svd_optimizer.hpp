// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// SVD-based rigid transform computation (Arun et al., 1987).
// Do not include this file directly; include <nanopcl/registration/icp.hpp>
//
// Optimization techniques:
//   1. OpenMP parallelization for centroid and covariance computation
//   2. Thread-local buffers with lock-free reduction
//   3. Stack-based buffers for L1 cache locality

#ifndef NANOPCL_REGISTRATION_IMPL_SVD_OPTIMIZER_HPP
#define NANOPCL_REGISTRATION_IMPL_SVD_OPTIMIZER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <array>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/registration/correspondence.hpp"

namespace npcl {
namespace registration {
namespace detail {

/**
 * @brief Compute optimal rigid transform using SVD (Parallelized)
 *
 * Finds the transformation T that minimizes:
 *   E = sum_i || T * src_i - tgt_i ||^2
 *
 * @param source Source point cloud
 * @param target Target point cloud
 * @param T_current Current transformation estimate (applied to source)
 * @param correspondences Valid correspondences
 * @return Incremental transformation delta_T such that T_new = delta_T *
 * T_current
 */
inline Eigen::Isometry3d computeOptimalTransformSVD(
    const PointCloud& source, const PointCloud& target,
    const Eigen::Isometry3d& T_current,
    const Correspondences& correspondences) {
  const size_t n = correspondences.size();

  // Global accumulators
  // Centroid: [src_x, src_y, src_z, tgt_x, tgt_y, tgt_z] = 6 doubles
  // Covariance H: 3x3 = 9 doubles (row-major: H00, H01, H02, H10, ...)
  double centroid_global[6] = {0};
  double H_global[9] = {0};

#ifdef _OPENMP
  const int num_threads = omp_get_max_threads();

  // Thread-local buffers: 6 (centroid) + 9 (H) = 15 doubles per thread
  std::vector<std::array<double, 15>> locals(num_threads);
  for (auto& arr : locals) arr.fill(0.0);

    // ==========================================================================
    // Pass 1: Compute centroids (PARALLEL)
    // ==========================================================================
#pragma omp parallel
  {
    const int tid = omp_get_thread_num();
    double* centroid_local = locals[tid].data();  // [0..5]

#pragma omp for nowait
    for (size_t i = 0; i < n; ++i) {
      const auto& c = correspondences[i];

      Eigen::Vector3d src_pt = T_current * source[c.source_idx].cast<double>();
      Eigen::Vector3d tgt_pt = target[c.target_idx].cast<double>();

      centroid_local[0] += src_pt.x();
      centroid_local[1] += src_pt.y();
      centroid_local[2] += src_pt.z();
      centroid_local[3] += tgt_pt.x();
      centroid_local[4] += tgt_pt.y();
      centroid_local[5] += tgt_pt.z();
    }
  }

  // Sequential reduction for centroids
  for (int t = 0; t < num_threads; ++t) {
    const double* centroid_local = locals[t].data();
    for (int k = 0; k < 6; ++k) {
      centroid_global[k] += centroid_local[k];
    }
  }

  // Compute mean
  const double inv_n = 1.0 / static_cast<double>(n);
  Eigen::Vector3d src_centroid(centroid_global[0] * inv_n,
                               centroid_global[1] * inv_n,
                               centroid_global[2] * inv_n);
  Eigen::Vector3d tgt_centroid(centroid_global[3] * inv_n,
                               centroid_global[4] * inv_n,
                               centroid_global[5] * inv_n);

  // Reset thread-local H buffers
  for (auto& arr : locals) {
    std::fill(arr.begin() + 6, arr.end(), 0.0);
  }

  // ==========================================================================
  // Pass 2: Compute covariance matrix H (PARALLEL)
  // ==========================================================================
#pragma omp parallel
  {
    const int tid = omp_get_thread_num();
    double* H_local = locals[tid].data() + 6;  // [6..14]

#pragma omp for nowait
    for (size_t i = 0; i < n; ++i) {
      const auto& c = correspondences[i];

      Eigen::Vector3d src_pt = T_current * source[c.source_idx].cast<double>();
      Eigen::Vector3d tgt_pt = target[c.target_idx].cast<double>();

      Eigen::Vector3d src_centered = src_pt - src_centroid;
      Eigen::Vector3d tgt_centered = tgt_pt - tgt_centroid;

      // H += src_centered * tgt_centered^T (row-major)
      H_local[0] += src_centered.x() * tgt_centered.x();
      H_local[1] += src_centered.x() * tgt_centered.y();
      H_local[2] += src_centered.x() * tgt_centered.z();
      H_local[3] += src_centered.y() * tgt_centered.x();
      H_local[4] += src_centered.y() * tgt_centered.y();
      H_local[5] += src_centered.y() * tgt_centered.z();
      H_local[6] += src_centered.z() * tgt_centered.x();
      H_local[7] += src_centered.z() * tgt_centered.y();
      H_local[8] += src_centered.z() * tgt_centered.z();
    }
  }

  // Sequential reduction for H
  for (int t = 0; t < num_threads; ++t) {
    const double* H_local = locals[t].data() + 6;
    for (int k = 0; k < 9; ++k) {
      H_global[k] += H_local[k];
    }
  }

#else
  // ==========================================================================
  // Sequential version (no OpenMP)
  // ==========================================================================

  // Pass 1: Compute centroids
  for (size_t i = 0; i < n; ++i) {
    const auto& c = correspondences[i];

    Eigen::Vector3d src_pt = T_current * source[c.source_idx].cast<double>();
    Eigen::Vector3d tgt_pt = target[c.target_idx].cast<double>();

    centroid_global[0] += src_pt.x();
    centroid_global[1] += src_pt.y();
    centroid_global[2] += src_pt.z();
    centroid_global[3] += tgt_pt.x();
    centroid_global[4] += tgt_pt.y();
    centroid_global[5] += tgt_pt.z();
  }

  const double inv_n = 1.0 / static_cast<double>(n);
  Eigen::Vector3d src_centroid(centroid_global[0] * inv_n,
                               centroid_global[1] * inv_n,
                               centroid_global[2] * inv_n);
  Eigen::Vector3d tgt_centroid(centroid_global[3] * inv_n,
                               centroid_global[4] * inv_n,
                               centroid_global[5] * inv_n);

  // Pass 2: Compute covariance matrix H
  for (size_t i = 0; i < n; ++i) {
    const auto& c = correspondences[i];

    Eigen::Vector3d src_pt = T_current * source[c.source_idx].cast<double>();
    Eigen::Vector3d tgt_pt = target[c.target_idx].cast<double>();

    Eigen::Vector3d src_centered = src_pt - src_centroid;
    Eigen::Vector3d tgt_centered = tgt_pt - tgt_centroid;

    H_global[0] += src_centered.x() * tgt_centered.x();
    H_global[1] += src_centered.x() * tgt_centered.y();
    H_global[2] += src_centered.x() * tgt_centered.z();
    H_global[3] += src_centered.y() * tgt_centered.x();
    H_global[4] += src_centered.y() * tgt_centered.y();
    H_global[5] += src_centered.y() * tgt_centered.z();
    H_global[6] += src_centered.z() * tgt_centered.x();
    H_global[7] += src_centered.z() * tgt_centered.y();
    H_global[8] += src_centered.z() * tgt_centered.z();
  }
#endif

  // ==========================================================================
  // Reconstruct 3x3 covariance matrix from flat array
  // ==========================================================================
  Eigen::Matrix3d H;
  H << H_global[0], H_global[1], H_global[2], H_global[3], H_global[4],
      H_global[5], H_global[6], H_global[7], H_global[8];

  // ==========================================================================
  // SVD decomposition: H = U * S * V^T
  // ==========================================================================
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // R = V * U^T
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();

  // Handle reflection case (det(R) = -1)
  if (R.determinant() < 0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1;
    R = V * svd.matrixU().transpose();
  }

  // t = tgt_centroid - R * src_centroid
  Eigen::Vector3d t = tgt_centroid - R * src_centroid;

  // ==========================================================================
  // Construct incremental transform
  // ==========================================================================
  // R, t align src_transformed (= T_current * source) to target.
  // This IS the incremental delta_T such that:
  //   delta_T * (T_current * source) ≈ target
  //   => T_total = delta_T * T_current
  Eigen::Isometry3d delta_T = Eigen::Isometry3d::Identity();
  delta_T.linear() = R;
  delta_T.translation() = t;

  return delta_T;
}

}  // namespace detail
}  // namespace registration
}  // namespace npcl

#endif  // NANOPCL_REGISTRATION_IMPL_SVD_OPTIMIZER_HPP
