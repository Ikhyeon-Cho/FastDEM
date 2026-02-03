// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Generic iterative solver for point cloud registration.
// Handles the optimization loop; context generation is delegated to caller.

#ifndef NANOPCL_REGISTRATION_ITERATIVE_SOLVER_HPP
#define NANOPCL_REGISTRATION_ITERATIVE_SOLVER_HPP

#include <Eigen/Geometry>
#include <cmath>
#include <limits>

#include "nanopcl/core/lie.hpp"
#include "nanopcl/registration/criteria.hpp"
#include "nanopcl/registration/linearizers/linearizer.hpp"
#include "nanopcl/registration/optimizers/gn_optimizer.hpp"
#include "nanopcl/registration/optimizers/lm_optimizer.hpp"
#include "nanopcl/registration/result.hpp"

namespace nanopcl {
namespace registration {

/// @brief Generic iterative solver for registration
///
/// This class handles only the optimization loop (Gauss-Newton / LM iterations).
/// Context generation (correspondence search, covariance handling) is delegated
/// to the caller via the linearize_func parameter.
///
/// @tparam Optimizer Optimizer type (LMOptimizer, GNOptimizer)
/// @tparam Linearizer Linearizer type (SerialLinearizer, ParallelLinearizer)
///
/// Example usage:
/// @code
///   IterativeSolver<LMOptimizer> solver;
///   solver.setMaxIterations(50);
///   solver.criteria().translation_eps = 1e-4;
///
///   auto result = solver.solve(n_points, [&](size_t idx, double* H, double* b, double* e) {
///     auto ctx = correspondence.find(transform(source[idx]), max_dist_sq);
///     if (!ctx) return false;
///     ICPFactor::linearize(*ctx, setting, H, b, e);
///     return true;
///   }, initial_guess);
/// @endcode
template <typename Optimizer = LMOptimizer,
          typename Linearizer = DefaultLinearizer>
class IterativeSolver {
public:
  // =========================================================================
  // Configuration
  // =========================================================================

  /// @brief Set maximum iterations
  void setMaxIterations(int max_iter) { criteria_.max_iterations = max_iter; }

  /// @brief Get maximum iterations
  [[nodiscard]] int maxIterations() const { return criteria_.max_iterations; }

  /// @brief Set convergence thresholds
  void setConvergenceThresholds(double translation_eps, double rotation_eps) {
    criteria_.translation_eps = translation_eps;
    criteria_.rotation_eps = rotation_eps;
  }

  /// @brief Set minimum correspondences required
  void setMinCorrespondences(size_t min_corr) {
    criteria_.min_correspondences = min_corr;
  }

  /// @brief Access termination criteria for advanced configuration
  TerminationCriteria& criteria() { return criteria_; }
  const TerminationCriteria& criteria() const { return criteria_; }

  /// @brief Access optimizer for advanced configuration
  Optimizer& optimizer() { return optimizer_; }
  const Optimizer& optimizer() const { return optimizer_; }

  /// @brief Access linearizer for advanced configuration
  Linearizer& linearizer() { return linearizer_; }
  const Linearizer& linearizer() const { return linearizer_; }

  // =========================================================================
  // Solve
  // =========================================================================

  /// @brief Run iterative optimization
  ///
  /// @tparam LinearizeFunc Callable: (size_t idx, const Isometry3d& T, double* H, double* b, double* e) -> bool
  /// @param n_points Number of source points
  /// @param linearize_func Function that linearizes one point given current transform.
  ///        Called as: linearize_func(idx, T_current, H_out, b_out, e_out)
  ///        Should return true if valid correspondence found.
  /// @param initial_guess Initial transformation estimate
  /// @return Registration result
  template <typename LinearizeFunc>
  [[nodiscard]] RegistrationResult solve(
      size_t n_points,
      LinearizeFunc&& linearize_func,
      const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity()) {
    if (n_points == 0) {
      return makeFailedResult(initial_guess, 0);
    }

    // Initialize transformation
    Eigen::Isometry3d T_current = initial_guess;
    double prev_error = std::numeric_limits<double>::max();
    double lambda = getLambda();
    Matrix6d last_H = Matrix6d::Zero();
    QuadraticModel last_model;

    // Iteration loop
    for (int iter = 0; iter < criteria_.max_iterations; ++iter) {
      // Linearize all points (pass T_current to linearize_func)
      QuadraticModel model =
          linearizer_.linearize(n_points, linearize_func, T_current);
      last_model = model;

      // Check correspondence count
      if (!criteria_.hasEnoughCorrespondences(model.num_inliers)) {
        return makeFailedResult(T_current, iter);
      }

      // Get full matrices
      Matrix6d H = model.toFullHessian();
      Vector6d b = model.toGradient();
      last_H = H;

      // Compute delta
      Vector6d delta = optimizer_.computeDelta(H, b, lambda);

      // Update transformation using Lie algebra
      Eigen::Isometry3d new_T = se3Exp(delta) * T_current;

      // Check convergence
      double curr_error = model.error;
      if (criteria_.converged(delta, prev_error, curr_error)) {
        return makeSuccessResult(new_T, model, iter + 1, last_H, n_points);
      }

      // Accept update
      T_current = new_T;
      prev_error = curr_error;
    }

    // Max iterations reached - return with last model's statistics
    return makeResult(T_current, last_model, criteria_.max_iterations, false,
                      last_H, n_points);
  }

protected:
  /// @brief Get lambda from optimizer (for LM) or default
  [[nodiscard]] double getLambda() const {
    if constexpr (std::is_same_v<Optimizer, LMOptimizer>) {
      return optimizer_.init_lambda;
    } else {
      return optimizer_.lambda;
    }
  }

  /// @brief Create failed result
  [[nodiscard]] RegistrationResult
  makeFailedResult(const Eigen::Isometry3d& T, int iterations) const {
    return {T,
            0.0,
            std::numeric_limits<double>::infinity(),
            static_cast<size_t>(iterations),
            false,
            std::nullopt};
  }

  /// @brief Create success result
  [[nodiscard]] RegistrationResult
  makeSuccessResult(const Eigen::Isometry3d& T,
                    const QuadraticModel& model,
                    int iterations,
                    const Matrix6d& H,
                    size_t n_points) const {
    // Fitness = fraction of source points with valid correspondences
    double fitness = (n_points > 0)
                         ? static_cast<double>(model.num_inliers) /
                               static_cast<double>(n_points)
                         : 0.0;
    double rmse = std::sqrt(2.0 * model.error / model.num_inliers);

    // Compute covariance from Hessian inverse
    std::optional<Matrix6d> cov;
    auto ldlt = H.ldlt();
    if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
      cov = ldlt.solve(Matrix6d::Identity());
    }

    return {T, fitness, rmse, static_cast<size_t>(iterations), true, cov};
  }

  /// @brief Create general result with model statistics
  [[nodiscard]] RegistrationResult makeResult(const Eigen::Isometry3d& T,
                                              const QuadraticModel& model,
                                              int iterations,
                                              bool converged,
                                              const Matrix6d& H,
                                              size_t n_points) const {
    std::optional<Matrix6d> cov;
    auto ldlt = H.ldlt();
    if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
      cov = ldlt.solve(Matrix6d::Identity());
    }

    // Compute fitness from model
    double fitness = (n_points > 0)
                         ? static_cast<double>(model.num_inliers) /
                               static_cast<double>(n_points)
                         : 0.0;
    double rmse = (model.num_inliers > 0)
                      ? std::sqrt(2.0 * model.error / model.num_inliers)
                      : std::numeric_limits<double>::infinity();

    return {T, fitness, rmse, static_cast<size_t>(iterations), converged, cov};
  }

protected:
  TerminationCriteria criteria_;
  Optimizer optimizer_;
  Linearizer linearizer_;
};

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_ITERATIVE_SOLVER_HPP
