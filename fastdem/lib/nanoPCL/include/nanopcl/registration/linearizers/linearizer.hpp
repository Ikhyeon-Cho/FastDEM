// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Linearizer for registration - builds normal equations from factors.
//
// Parallelization:
//   - Automatically uses OpenMP if compiled with -fopenmp
//   - For explicit thread control, use <nanopcl/registration/linearizers/linearizer_omp.hpp>

#ifndef NANOPCL_REGISTRATION_LINEARIZERS_LINEARIZER_HPP
#define NANOPCL_REGISTRATION_LINEARIZERS_LINEARIZER_HPP

#include <array>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>

#include "nanopcl/registration/result.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace nanopcl {
namespace registration {

// =============================================================================
// Upper-triangular Hessian Index Convention
// =============================================================================

/// Upper-triangular 6x6 packing (row-major, 21 elements)
/// Layout: [H00, H01, H02, H03, H04, H05,
///               H11, H12, H13, H14, H15,
///                    H22, H23, H24, H25,
///                         H33, H34, H35,
///                              H44, H45,
///                                   H55]

/// @brief Convert upper-triangular array to full 6x6 symmetric matrix
inline Matrix6d toFullHessian(const double* H_upper) {
  Matrix6d H;
  int idx = 0;
  for (int row = 0; row < 6; ++row) {
    for (int col = row; col < 6; ++col) {
      H(row, col) = H_upper[idx];
      H(col, row) = H_upper[idx]; // Symmetric fill
      ++idx;
    }
  }
  return H;
}

/// @brief Convert array to Vector6d
inline Vector6d toVector6d(const double* b) {
  return Eigen::Map<const Vector6d>(b);
}

// =============================================================================
// Quadratic Model (formerly LinearizedSystem)
// =============================================================================

/// @brief Accumulated quadratic approximation from all factors
///
/// Represents: E(x) ≈ 0.5 * x^T * H * x + b^T * x + c
/// where H is the Hessian, b is the gradient, and c is the constant (error).
struct QuadraticModel {
  double H[21] = {0};     ///< Upper-triangular Hessian (21 elements)
  double b[6] = {0};      ///< Gradient vector (6 elements)
  double error = 0.0;     ///< Total error (constant term)
  size_t num_inliers = 0; ///< Number of valid correspondences

  /// @brief Accumulate another model into this one
  void accumulate(const QuadraticModel& other) {
    for (int i = 0; i < 21; ++i)
      H[i] += other.H[i];
    for (int i = 0; i < 6; ++i)
      b[i] += other.b[i];
    error += other.error;
    num_inliers += other.num_inliers;
  }

  /// @brief Reset to zero
  void reset() {
    for (int i = 0; i < 21; ++i)
      H[i] = 0;
    for (int i = 0; i < 6; ++i)
      b[i] = 0;
    error = 0.0;
    num_inliers = 0;
  }

  /// @brief Get full 6x6 Hessian matrix
  [[nodiscard]] Matrix6d toFullHessian() const {
    return nanopcl::registration::toFullHessian(H);
  }

  /// @brief Get gradient vector
  [[nodiscard]] Vector6d toGradient() const {
    return nanopcl::registration::toVector6d(b);
  }
};

// =============================================================================
// Serial Linearizer (formerly SerialReduction)
// =============================================================================

/// @brief Single-threaded linearizer
///
/// Use this for debugging or when data size is small.
/// For large point clouds, use ParallelLinearizer or DefaultLinearizer.
struct SerialLinearizer {
  /// @brief Linearize all points and accumulate
  /// @param n Number of points to process
  /// @param linearize_func Function: (idx, T, H*, b*, e*) -> bool
  /// @param T_current Current transformation estimate
  /// @return Accumulated quadratic model
  template <typename LinearizeFunc>
  [[nodiscard]] QuadraticModel linearize(
      size_t n,
      LinearizeFunc&& linearize_func,
      const Eigen::Isometry3d& T_current) const {
    QuadraticModel result;
    double H_point[21], b_point[6], e_point;

    for (size_t i = 0; i < n; ++i) {
      // Zero-initialize point buffers
      for (int k = 0; k < 21; ++k)
        H_point[k] = 0;
      for (int k = 0; k < 6; ++k)
        b_point[k] = 0;
      e_point = 0;

      if (linearize_func(i, T_current, H_point, b_point, &e_point)) {
        for (int k = 0; k < 21; ++k)
          result.H[k] += H_point[k];
        for (int k = 0; k < 6; ++k)
          result.b[k] += b_point[k];
        result.error += e_point;
        ++result.num_inliers;
      }
    }

    return result;
  }
};

// =============================================================================
// Parallel Linearizer (auto-enabled with OpenMP)
// =============================================================================

/// @brief OpenMP parallel linearizer with thread-local buffers
///
/// Uses lock-free accumulation pattern for high performance.
/// Each thread accumulates into its own buffer, then results are merged.
///
/// Automatically used when OpenMP is available (-fopenmp).
struct ParallelLinearizer {
  /// @brief Linearize all points in parallel and accumulate
  /// @param n Number of points to process
  /// @param linearize_func Function: (idx, T, H*, b*, e*) -> bool
  /// @param T_current Current transformation estimate
  /// @return Accumulated quadratic model
  template <typename LinearizeFunc>
  [[nodiscard]] QuadraticModel linearize(
      size_t n,
      LinearizeFunc&& linearize_func,
      const Eigen::Isometry3d& T_current) const {
#ifdef _OPENMP
    const int nthreads = omp_get_max_threads();

    // Thread-local buffers: 21 (H) + 6 (b) + 1 (error) + 1 (count) = 29 doubles
    std::vector<std::array<double, 29>> locals(nthreads);
    for (auto& arr : locals)
      arr.fill(0.0);

#pragma omp parallel
    {
      const int tid = omp_get_thread_num();
      double* H_local = locals[tid].data();       // [0..20]
      double* b_local = locals[tid].data() + 21;  // [21..26]
      double& e_local = locals[tid][27];
      double& count_local = locals[tid][28];

      double H_point[21], b_point[6], e_point;

#pragma omp for nowait schedule(dynamic, 64)
      for (size_t i = 0; i < n; ++i) {
        // Zero-initialize point buffers
        for (int k = 0; k < 21; ++k)
          H_point[k] = 0;
        for (int k = 0; k < 6; ++k)
          b_point[k] = 0;
        e_point = 0;

        if (linearize_func(i, T_current, H_point, b_point, &e_point)) {
          for (int k = 0; k < 21; ++k)
            H_local[k] += H_point[k];
          for (int k = 0; k < 6; ++k)
            b_local[k] += b_point[k];
          e_local += e_point;
          count_local += 1.0;
        }
      }
    }

    // Sequential reduction (no lock contention)
    QuadraticModel result;
    for (int t = 0; t < nthreads; ++t) {
      for (int k = 0; k < 21; ++k)
        result.H[k] += locals[t][k];
      for (int k = 0; k < 6; ++k)
        result.b[k] += locals[t][21 + k];
      result.error += locals[t][27];
      result.num_inliers += static_cast<size_t>(locals[t][28]);
    }

    return result;
#else
    // Fallback to serial when OpenMP not available
    SerialLinearizer serial;
    return serial.linearize(n, std::forward<LinearizeFunc>(linearize_func),
                            T_current);
#endif
  }
};

// =============================================================================
// Default Linearizer (auto-select based on OpenMP availability)
// =============================================================================

#ifdef _OPENMP
using DefaultLinearizer = ParallelLinearizer;
#else
using DefaultLinearizer = SerialLinearizer;
#endif

} // namespace registration
} // namespace nanopcl

#endif // NANOPCL_REGISTRATION_LINEARIZERS_LINEARIZER_HPP
