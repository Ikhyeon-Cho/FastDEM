// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_REGISTRATION_OPTIMIZER_GN_OPTIMIZER_HPP
#define NANOPCL_REGISTRATION_OPTIMIZER_GN_OPTIMIZER_HPP

#include <Eigen/Dense>
#include <iostream>

#include "nanopcl/core/lie.hpp"
#include "nanopcl/registration/result.hpp"

namespace nanopcl {
namespace registration {

/// @brief Gauss-Newton optimizer for registration
///
/// Simple and fast, but may diverge with poor initial guesses.
/// Use LMOptimizer for more robust convergence.
struct GNOptimizer {
  /// Small regularization for numerical stability
  double lambda = 1e-6;

  /// Verbose output
  bool verbose = false;

  /// @brief Solve one iteration of GN optimization
  /// @param H 6x6 Hessian matrix (information matrix)
  /// @param b 6x1 gradient vector
  /// @param current_T Current transformation estimate
  /// @return Updated transformation
  [[nodiscard]] Eigen::Isometry3d solve(const Matrix6d& H,
                                        const Vector6d& b,
                                        const Eigen::Isometry3d& current_T) const {
    // Solve with small regularization: (H + lambda * I) * delta = -b
    Matrix6d H_reg = H + lambda * Matrix6d::Identity();
    Vector6d delta = H_reg.ldlt().solve(-b);

    if (verbose) {
      std::cerr << "  GN dt=" << delta.tail<3>().norm()
                << " dr=" << delta.head<3>().norm() << "\n";
    }

    // Apply update: T_new = exp(delta) * T_current
    return se3Exp(delta) * current_T;
  }

  /// @brief Get the delta vector from H and b (for convergence check)
  [[nodiscard]] Vector6d computeDelta(const Matrix6d& H,
                                      const Vector6d& b) const {
    Matrix6d H_reg = H + lambda * Matrix6d::Identity();
    return H_reg.ldlt().solve(-b);
  }

  /// @brief Compute only the transformation update (delta)
  [[nodiscard]] Eigen::Isometry3d computeUpdate(const Matrix6d& H,
                                                const Vector6d& b) const {
    Vector6d delta = computeDelta(H, b);
    return se3Exp(delta);
  }
};

} // namespace registration
} // namespace nanopcl

#endif // NANOPCL_REGISTRATION_OPTIMIZER_GN_OPTIMIZER_HPP
