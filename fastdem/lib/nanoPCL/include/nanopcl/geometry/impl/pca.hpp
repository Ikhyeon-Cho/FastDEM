// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// PCA computation helpers with numerical stability.

#ifndef NANOPCL_GEOMETRY_IMPL_PCA_HPP
#define NANOPCL_GEOMETRY_IMPL_PCA_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <vector>

#include "nanopcl/core/types.hpp"

namespace nanopcl {
namespace geometry {
namespace detail {

/// @brief PCA result containing eigenvalues and eigenvectors
struct PCAResult {
  Eigen::Vector3f eigenvalues;  // Ascending order (smallest first)
  Eigen::Matrix3f eigenvectors; // Column i corresponds to eigenvalue i
  bool valid = false;
};

/// @brief Compute mean and covariance from neighbor points (1-Pass with offsetting)
///
/// Uses the first point as offset to improve numerical stability.
/// Formula: Cov = (sum_sq - sum * sum^T / n) / n
///
/// @param cloud Point cloud
/// @param indices Neighbor indices
/// @return Pair of (mean, covariance)
template <typename PointCloud>
inline std::pair<Eigen::Vector3f, Eigen::Matrix3f> computeMeanAndCovariance(
    const PointCloud& cloud,
    const std::vector<uint32_t>& indices) {

  const size_t n = indices.size();

  // Use first point as offset for numerical stability
  const Eigen::Vector3f offset = cloud.point(indices[0]);

  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  Eigen::Matrix3f sum_sq = Eigen::Matrix3f::Zero();

  for (uint32_t idx : indices) {
    const Eigen::Vector3f d = cloud.point(idx) - offset;
    sum += d;
    sum_sq.noalias() += d * d.transpose();
  }

  const float inv_n = 1.0f / static_cast<float>(n);
  const Eigen::Vector3f mean = offset + sum * inv_n;
  const Eigen::Matrix3f cov = (sum_sq - sum * sum.transpose() * inv_n) * inv_n;

  return {mean, cov};
}

/// @brief Perform PCA on covariance matrix
///
/// Uses Eigen's SelfAdjointEigenSolver with computeDirect for 3x3 optimization.
/// Returns eigenvalues in ascending order (smallest = normal direction).
///
/// @param cov 3x3 covariance matrix
/// @return PCA result with eigenvalues and eigenvectors
inline PCAResult computePCA(const Eigen::Matrix3f& cov) {
  PCAResult result;

  // Check for degenerate covariance
  const float trace = cov.trace();
  if (trace < std::numeric_limits<float>::epsilon()) {
    return result; // valid = false
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver;
  solver.computeDirect(cov);

  if (solver.info() != Eigen::Success) {
    return result; // valid = false
  }

  result.eigenvalues = solver.eigenvalues();
  result.eigenvectors = solver.eigenvectors();
  result.valid = true;

  return result;
}

/// @brief Compute PCA directly from neighbor points
///
/// Combines mean/covariance computation and eigendecomposition.
/// Returns invalid result if fewer than 3 points.
///
/// @param cloud Point cloud
/// @param indices Neighbor indices (must have at least 3 elements)
/// @return PCA result
template <typename PointCloud>
inline PCAResult computePCA(const PointCloud& cloud,
                            const std::vector<uint32_t>& indices) {
  if (indices.size() < 3) {
    return PCAResult{}; // valid = false
  }

  auto [mean, cov] = computeMeanAndCovariance(cloud, indices);
  return computePCA(cov);
}

} // namespace detail
} // namespace geometry
} // namespace nanopcl

#endif // NANOPCL_GEOMETRY_IMPL_PCA_HPP
