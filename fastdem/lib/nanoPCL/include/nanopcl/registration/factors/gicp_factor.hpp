// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// GICP (Generalized ICP) factor for distribution-to-distribution alignment.
// Reference: Segal, Haehnel, Thrun. "Generalized-ICP" (RSS 2009)

#ifndef NANOPCL_REGISTRATION_FACTORS_GICP_FACTOR_HPP
#define NANOPCL_REGISTRATION_FACTORS_GICP_FACTOR_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/registration/context.hpp"
#include "nanopcl/registration/factors/robust_kernels.hpp"

namespace nanopcl {
namespace registration {

// =============================================================================
// GICP Factor Utilities
// =============================================================================

namespace detail {

/// @brief Analytical 3x3 matrix inverse using Cramer's rule
/// Faster than Eigen::inverse() for small matrices.
inline Eigen::Matrix3d inverse3x3(const Eigen::Matrix3d& M) {
  const double det = M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) -
                     M(0, 1) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0)) +
                     M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0));

  const double inv_det = 1.0 / det;

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

/// @brief Regularize covariance matrix for GICP
/// Normalizes eigenvalues to [epsilon, 1, 1] for plane-to-plane alignment.
inline Eigen::Matrix3d regularizeCovariance(const Eigen::Matrix3f& cov,
                                            double epsilon) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov.cast<double>());

  // Normalize per Segal et al. (RSS 2009): [epsilon, 1, 1]
  Eigen::Vector3d eigenvalues;
  eigenvalues(0) = epsilon; // Smallest: plane normal direction
  eigenvalues(1) = 1.0;     // Tangent direction 1
  eigenvalues(2) = 1.0;     // Tangent direction 2

  return solver.eigenvectors() * eigenvalues.asDiagonal() *
         solver.eigenvectors().transpose();
}

} // namespace detail

// =============================================================================
// GICP Factor
// =============================================================================

/// @brief GICP factor for distribution-to-distribution alignment
///
/// Minimizes Mahalanobis distance:
///   e = 0.5 * r^T * W * r
/// where W = (C_src + C_tgt)^{-1}
struct GICPFactor {
  /// Factor settings
  struct Setting {
    RobustKernel robust_kernel = RobustKernel::NONE;
    double robust_kernel_width = 1.0;
  };

  /// @brief Linearize GICP cost for one correspondence
  ///
  /// Computes H += J^T * W * J, b += J^T * W * r, e += 0.5 * r^T * W * r
  ///
  /// @param ctx Distribution context with points and covariances
  /// @param setting Factor settings (robust kernel, etc.)
  /// @param H_out [out] 21-element upper-triangular Hessian contribution
  /// @param b_out [out] 6-element gradient contribution
  /// @param e_out [out] Error contribution
  static void linearize(const DistributionContext& ctx,
                        const Setting& setting,
                        double* H_out,
                        double* b_out,
                        double* e_out) {
    // Combined covariance: C = C_src + C_tgt
    // Note: C_src is already rotated (R * C_original * R^T) by Correspondence
    const Eigen::Matrix3d C = ctx.C_src + ctx.C_tgt;

    // Weight matrix: W = C^{-1}
    const Eigen::Matrix3d W = detail::inverse3x3(C);

    // Residual: r = p_src - p_tgt
    const double rx = ctx.p_src.x() - ctx.p_tgt.x();
    const double ry = ctx.p_src.y() - ctx.p_tgt.y();
    const double rz = ctx.p_src.z() - ctx.p_tgt.z();

    // Extract W elements (symmetric)
    double w00 = W(0, 0), w01 = W(0, 1), w02 = W(0, 2);
    double w11 = W(1, 1), w12 = W(1, 2), w22 = W(2, 2);

    // Compute W * r
    double wr0 = w00 * rx + w01 * ry + w02 * rz;
    double wr1 = w01 * rx + w11 * ry + w12 * rz;
    double wr2 = w02 * rx + w12 * ry + w22 * rz;

    // Mahalanobis distance for robust weighting
    const double mahal_sq = rx * wr0 + ry * wr1 + rz * wr2;
    const double mahal_dist = std::sqrt(std::max(mahal_sq, 0.0));
    const double rw = computeRobustWeight(mahal_dist, setting.robust_kernel, setting.robust_kernel_width);

    // Scale W and Wr by robust weight
    w00 *= rw;
    w01 *= rw;
    w02 *= rw;
    w11 *= rw;
    w12 *= rw;
    w22 *= rw;
    wr0 *= rw;
    wr1 *= rw;
    wr2 *= rw;

    // Transformed point coordinates
    const double tx = ctx.p_src.x();
    const double ty = ctx.p_src.y();
    const double tz = ctx.p_src.z();

    // b = [Wr; p_src × Wr] (6-vector gradient)
    const double b3 = ty * wr2 - tz * wr1;
    const double b4 = tz * wr0 - tx * wr2;
    const double b5 = tx * wr1 - ty * wr0;

    // H = J^T * W * J
    // H_12 = -W * [p_src]×
    const double h03 = -w01 * tz + w02 * ty;
    const double h04 = w00 * tz - w02 * tx;
    const double h05 = -w00 * ty + w01 * tx;
    const double h13 = -w11 * tz + w12 * ty;
    const double h14 = w01 * tz - w12 * tx;
    const double h15 = -w01 * ty + w11 * tx;
    const double h23 = -w12 * tz + w22 * ty;
    const double h24 = w02 * tz - w22 * tx;
    const double h25 = -w02 * ty + w12 * tx;

    // H_22 = -[p_src]× * W * [p_src]×
    const double h33 = -(tz * h13 - ty * h23);
    const double h34 = -(tz * h14 - ty * h24);
    const double h35 = -(tz * h15 - ty * h25);
    const double h44 = -(-tz * h04 + tx * h24);
    const double h45 = -(-tz * h05 + tx * h25);
    const double h55 = -(ty * h05 - tx * h15);

    // Accumulate upper-triangular H (21 elements)
    // H_11 block (W, symmetric)
    H_out[0] += w00;
    H_out[1] += w01;
    H_out[2] += w02;
    H_out[6] += w11;
    H_out[7] += w12;
    H_out[11] += w22;

    // H_12 block
    H_out[3] += h03;
    H_out[4] += h04;
    H_out[5] += h05;
    H_out[8] += h13;
    H_out[9] += h14;
    H_out[10] += h15;
    H_out[12] += h23;
    H_out[13] += h24;
    H_out[14] += h25;

    // H_22 block
    H_out[15] += h33;
    H_out[16] += h34;
    H_out[17] += h35;
    H_out[18] += h44;
    H_out[19] += h45;
    H_out[20] += h55;

    // Accumulate gradient
    b_out[0] += wr0;
    b_out[1] += wr1;
    b_out[2] += wr2;
    b_out[3] += b3;
    b_out[4] += b4;
    b_out[5] += b5;

    // Error: 0.5 * r^T * W * r (with robust weight)
    *e_out += 0.5 * rw * mahal_sq;
  }
};

// =============================================================================
// GICP Covariance Pre-computation
// =============================================================================

/// @brief Pre-compute regularized covariances for all points
/// @param cloud Point cloud with raw covariances
/// @param epsilon Regularization value (smallest eigenvalue)
/// @return Vector of regularized 3x3 covariance matrices
inline std::vector<Eigen::Matrix3d> precomputeRegularizedCovariances(
    const PointCloud& cloud,
    double epsilon = 1e-3) {
  const size_t n = cloud.size();
  std::vector<Eigen::Matrix3d> regularized(n);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (size_t i = 0; i < n; ++i) {
    const auto& cov = cloud.covariance(i);
    if (std::isfinite(cov(0, 0))) {
      regularized[i] = detail::regularizeCovariance(cov, epsilon);
    } else {
      // Invalid covariance - use identity scaled by epsilon
      regularized[i] = Eigen::Matrix3d::Identity() * epsilon;
    }
  }

  return regularized;
}

} // namespace registration
} // namespace nanopcl

#endif // NANOPCL_REGISTRATION_FACTORS_GICP_FACTOR_HPP
