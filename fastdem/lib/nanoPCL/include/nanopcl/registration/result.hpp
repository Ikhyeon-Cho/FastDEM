// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_REGISTRATION_RESULT_HPP
#define NANOPCL_REGISTRATION_RESULT_HPP

#include <Eigen/Geometry>
#include <cstddef>
#include <optional>

namespace nanopcl {
namespace registration {

/// Type alias for 6x6 covariance matrix [rotation; translation]
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Type alias for 6-element vector [rotation; translation]
using Vector6d = Eigen::Matrix<double, 6, 1>;

/**
 * @brief Result of point cloud registration
 *
 * Contains the computed transformation, quality metrics, and convergence
 * status.
 */
struct RegistrationResult {
  Eigen::Isometry3d transform; ///< Final transformation (source -> target)
  double fitness;              ///< Inlier ratio [0, 1]
  double rmse;                 ///< Root mean squared error of inliers
  size_t iterations;           ///< Number of iterations performed
  bool converged;              ///< Whether algorithm converged

  /// 6x6 covariance matrix of the transformation estimate [t; omega] order.
  /// Only available for iterative methods (ICP-Plane, GICP, VGICP).
  /// nullopt if not computed or if Hessian was singular.
  std::optional<Matrix6d> covariance;

  /**
   * @brief Check if registration succeeded
   * @param min_fitness Minimum acceptable fitness ratio (default: 0.5)
   * @return true if converged with fitness >= min_fitness
   */
  [[nodiscard]] bool success(double min_fitness = 0.5) const noexcept {
    return converged && fitness >= min_fitness;
  }

  /**
   * @brief Get information matrix (inverse of covariance)
   *
   * Useful for pose graph optimization where information matrices are needed.
   *
   * @return Information matrix if covariance exists, nullopt otherwise
   */
  [[nodiscard]] std::optional<Matrix6d> informationMatrix() const {
    if (!covariance)
      return std::nullopt;
    return covariance->inverse();
  }
};

} // namespace registration
} // namespace nanopcl

#endif // NANOPCL_REGISTRATION_RESULT_HPP
