// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// OpenMP-parallelized linearizer with explicit thread control.
//
// Use this header when you need fine-grained control over parallelization:
//   - Specify exact number of threads
//   - Integrate with your own thread management
//
// For automatic parallelization, use <nanopcl/registration/linearizers/linearizer.hpp> instead.

#ifndef NANOPCL_REGISTRATION_LINEARIZERS_LINEARIZER_OMP_HPP
#define NANOPCL_REGISTRATION_LINEARIZERS_LINEARIZER_OMP_HPP

#include <omp.h>

#include <array>
#include <vector>

#include "nanopcl/registration/linearizers/linearizer.hpp"

namespace nanopcl {
namespace registration {

// =============================================================================
// OpenMP Parallel Linearizer with Explicit Thread Control
// =============================================================================

/// @brief OpenMP parallel linearizer with explicit thread control
///
/// Uses lock-free accumulation pattern for high performance.
/// Each thread accumulates into its own buffer, then results are merged.
///
/// @note Requires OpenMP. For automatic parallelization, use ParallelLinearizer
///       from linearizer.hpp instead.
struct ParallelLinearizerOMP {
  int num_threads = 4;  ///< Number of OpenMP threads

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
    const int nthreads = (num_threads > 0) ? num_threads : omp_get_max_threads();

    // Thread-local buffers: 21 (H) + 6 (b) + 1 (error) + 1 (count) = 29 doubles
    std::vector<std::array<double, 29>> locals(nthreads);
    for (auto& arr : locals)
      arr.fill(0.0);

#pragma omp parallel num_threads(nthreads)
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
  }
};

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_LINEARIZERS_LINEARIZER_OMP_HPP
