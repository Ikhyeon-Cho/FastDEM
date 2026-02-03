// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_REGISTRATION_OPTIMIZER_LM_OPTIMIZER_HPP
#define NANOPCL_REGISTRATION_OPTIMIZER_LM_OPTIMIZER_HPP

#include <Eigen/Dense>
#include <iostream>

#include "nanopcl/core/lie.hpp"
#include "nanopcl/registration/result.hpp"

namespace nanopcl {
namespace registration {

/// @brief Levenberg-Marquardt optimizer for registration
///
/// Adaptive damping: increases lambda when error increases, decreases when
/// error decreases. More robust than Gauss-Newton for poor initial guesses.
struct LMOptimizer {
  /// Initial damping factor
  double init_lambda = 1e-3;

  /// Lambda multiplier/divisor for adaptive adjustment
  double lambda_factor = 10.0;

  /// Maximum inner iterations for lambda adjustment per outer iteration
  int max_inner_iterations = 10;

  /// Verbose output
  bool verbose = false;

  /// @brief Solve one iteration of LM optimization
  /// @param H 6x6 Hessian matrix (information matrix)
  /// @param b 6x1 gradient vector
  /// @param current_T Current transformation estimate
  /// @param current_error Current total error
  /// @param lambda [in/out] Damping factor (modified based on success/failure)
  /// @param new_error [out] New error after update (for error evaluation)
  /// @return Transformation update delta (to be applied as delta * current_T)
  template <typename ErrorFunc>
  [[nodiscard]] Eigen::Isometry3d solve(const Matrix6d& H,
                                        const Vector6d& b,
                                        const Eigen::Isometry3d& current_T,
                                        double current_error,
                                        double& lambda,
                                        ErrorFunc&& compute_error) const {
    Eigen::Isometry3d best_T = current_T;
    double best_error = current_error;
    Vector6d best_delta = Vector6d::Zero();

    for (int inner = 0; inner < max_inner_iterations; ++inner) {
      // Solve damped normal equation: (H + lambda * I) * delta = -b
      Matrix6d H_damped = H + lambda * Matrix6d::Identity();
      Vector6d delta = H_damped.ldlt().solve(-b);

      // Compute new transform
      Eigen::Isometry3d new_T = se3Exp(delta) * current_T;

      // Evaluate new error
      double new_error = compute_error(new_T);

      if (verbose) {
        std::cerr << "  LM inner=" << inner << " lambda=" << lambda
                  << " error=" << current_error << " -> " << new_error
                  << " dt=" << delta.tail<3>().norm()
                  << " dr=" << delta.head<3>().norm() << "\n";
      }

      if (new_error < current_error) {
        // Success: decrease lambda, accept update
        lambda /= lambda_factor;
        best_T = new_T;
        best_error = new_error;
        best_delta = delta;
        break;
      } else {
        // Failure: increase lambda, try again
        lambda *= lambda_factor;
      }
    }

    return best_T;
  }

  /// @brief Simple solve without error evaluation (single step)
  /// @note Use this when error evaluation is expensive and you want GN-like
  /// behavior
  [[nodiscard]] Eigen::Isometry3d solveSingleStep(const Matrix6d& H,
                                                  const Vector6d& b,
                                                  double lambda) const {
    Matrix6d H_damped = H + lambda * Matrix6d::Identity();
    Vector6d delta = H_damped.ldlt().solve(-b);
    return se3Exp(delta);
  }

  /// @brief Get the delta vector from H and b (for convergence check)
  [[nodiscard]] Vector6d computeDelta(const Matrix6d& H,
                                      const Vector6d& b,
                                      double lambda) const {
    Matrix6d H_damped = H + lambda * Matrix6d::Identity();
    return H_damped.ldlt().solve(-b);
  }
};

} // namespace registration
} // namespace nanopcl

#endif // NANOPCL_REGISTRATION_OPTIMIZER_LM_OPTIMIZER_HPP
