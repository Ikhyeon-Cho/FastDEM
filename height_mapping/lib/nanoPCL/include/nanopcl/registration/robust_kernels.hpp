// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Robust Kernel functions for M-estimation in registration.
//
// These kernels reduce the influence of outliers by down-weighting
// large residuals. Used in IRLS (Iteratively Reweighted Least Squares)
// within ICP variants.
//
// Reference: Zhang, "Parameter Estimation Techniques: A Tutorial" (1997)

#ifndef NANOPCL_REGISTRATION_ROBUST_KERNELS_HPP
#define NANOPCL_REGISTRATION_ROBUST_KERNELS_HPP

#include <cmath>

namespace npcl {
namespace registration {

/**
 * @brief Available robust kernel types for M-estimation
 */
enum class RobustKernel {
  NONE,   ///< Standard L2 loss (no robustness)
  HUBER,  ///< Huber loss: L2 for small errors, L1 for large errors
  TUKEY,  ///< Tukey's biweight: hard cutoff for outliers (weight=0)
  CAUCHY  ///< Cauchy (Lorentzian): soft down-weighting
};

/**
 * @brief Compute robust weight for a given residual
 *
 * Returns w(r) = rho'(r) / r, where rho is the loss function.
 * This weight is used in IRLS: minimize sum_i w_i * r_i^2
 *
 * @param residual The residual value (can be negative)
 * @param kernel The robust kernel type
 * @param k The kernel width parameter (scale)
 * @return Weight in range [0, 1]
 */
inline double computeRobustWeight(double residual, RobustKernel kernel,
                                  double k) {
  const double r_abs = std::abs(residual);

  switch (kernel) {
    case RobustKernel::NONE:
      return 1.0;

    case RobustKernel::HUBER:
      // w(r) = 1 if |r| <= k, else k/|r|
      // Provides L2 behavior for small errors, L1 for large
      return (r_abs <= k) ? 1.0 : (k / r_abs);

    case RobustKernel::TUKEY: {
      // w(r) = (1 - (r/k)^2)^2 if |r| <= k, else 0
      // Completely ignores outliers beyond k (hard cutoff)
      if (r_abs > k) return 0.0;
      const double x = r_abs / k;
      const double t = 1.0 - x * x;
      return t * t;
    }

    case RobustKernel::CAUCHY: {
      // w(r) = 1 / (1 + (r/k)^2)
      // Soft down-weighting based on Lorentzian distribution
      const double x = r_abs / k;
      return 1.0 / (1.0 + x * x);
    }

    default:
      return 1.0;
  }
}

/**
 * @brief Recommended kernel width values
 *
 * These values provide ~95% efficiency for Gaussian noise.
 */
namespace kernel_defaults {
constexpr double HUBER_K = 1.345;   ///< 95% efficiency for Gaussian
constexpr double TUKEY_K = 4.685;   ///< 95% efficiency for Gaussian
constexpr double CAUCHY_K = 2.385;  ///< 95% efficiency for Gaussian
}  // namespace kernel_defaults

}  // namespace registration
}  // namespace npcl

#endif  // NANOPCL_REGISTRATION_ROBUST_KERNELS_HPP
