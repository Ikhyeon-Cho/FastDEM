// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_REGISTRATION_CRITERIA_HPP
#define NANOPCL_REGISTRATION_CRITERIA_HPP

#include <cmath>
#include <cstddef>

#include "nanopcl/registration/result.hpp"

namespace nanopcl {
namespace registration {

/// @brief Termination criteria for iterative registration
struct TerminationCriteria {
  /// Maximum number of iterations
  int max_iterations = 50;

  /// Minimum number of valid correspondences to continue
  size_t min_correspondences = 10;

  /// Translation convergence threshold (meters)
  /// Stop if |delta_translation| < translation_eps
  double translation_eps = 1e-4;

  /// Rotation convergence threshold (radians)
  /// Stop if |delta_rotation| < rotation_eps
  double rotation_eps = 1e-4;

  /// Relative error reduction threshold
  /// Stop if |e_prev - e_curr| / e_prev < relative_error_eps
  double relative_error_eps = 1e-6;

  /// @brief Check if optimization has converged
  /// @param delta 6-DOF update vector [rotation(3); translation(3)]
  /// @param prev_error Previous iteration's error
  /// @param curr_error Current iteration's error
  /// @return true if any convergence condition is met
  [[nodiscard]] bool converged(const Vector6d& delta,
                               double prev_error,
                               double curr_error) const noexcept {
    // Check 1: Delta is small (transformation is stable)
    const double dt = delta.tail<3>().norm();
    const double dr = delta.head<3>().norm();
    const bool is_stable = (dt < translation_eps) && (dr < rotation_eps);

    // Check 2: Relative error reduction is small (optimization stalled)
    const double rel_change =
        (prev_error > 0) ? std::abs(prev_error - curr_error) / prev_error : 0.0;
    const bool is_stalled = (rel_change < relative_error_eps);

    return is_stable || is_stalled;
  }

  /// @brief Check if iteration count is valid
  /// @param iter Current iteration number (0-indexed)
  /// @return true if more iterations are allowed
  [[nodiscard]] bool canContinue(int iter) const noexcept {
    return iter < max_iterations;
  }

  /// @brief Check if correspondence count is sufficient
  /// @param count Number of valid correspondences
  /// @return true if count is sufficient
  [[nodiscard]] bool hasEnoughCorrespondences(size_t count) const noexcept {
    return count >= min_correspondences;
  }
};

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_CRITERIA_HPP
