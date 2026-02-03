// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// OpenMP-parallelized normal and covariance estimation with explicit thread control.
//
// Use this header when you need fine-grained control over parallelization:
//   - Specify exact number of threads
//   - Integrate with your own thread management
//
// For automatic parallelization, use <nanopcl/geometry/normal_estimation.hpp> instead.

#ifndef NANOPCL_GEOMETRY_NORMAL_ESTIMATION_OMP_HPP
#define NANOPCL_GEOMETRY_NORMAL_ESTIMATION_OMP_HPP

#include <omp.h>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/geometry/impl/normal_estimation_impl.hpp"

namespace nanopcl {
namespace geometry {

/// @brief Estimate surface normals with explicit thread control
///
/// @param cloud       Point cloud (normal channel will be enabled)
/// @param k           Number of neighbors (default: 20)
/// @param num_threads Number of OpenMP threads (default: 4)
/// @param viewpoint   Viewpoint for normal orientation (default: origin)
inline void estimateNormalsOMP(PointCloud& cloud,
                               int k = 20,
                               int num_threads = 4,
                               const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return;

  detail::NormalSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalSetter::prepare(cloud);

  search::KdTree tree;
  tree.build(cloud);

  const size_t n = cloud.size();

#pragma omp parallel for num_threads(num_threads) schedule(dynamic, 64)
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate surface normals with external KdTree and explicit thread control
inline void estimateNormalsOMP(PointCloud& cloud,
                               const search::KdTree& tree,
                               int k = 20,
                               int num_threads = 4,
                               const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return;

  detail::NormalSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalSetter::prepare(cloud);

  const size_t n = cloud.size();

#pragma omp parallel for num_threads(num_threads) schedule(dynamic, 64)
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate GICP-ready covariances with explicit thread control
///
/// @param cloud       Point cloud (covariance channel will be enabled)
/// @param k           Number of neighbors (default: 20)
/// @param num_threads Number of OpenMP threads (default: 4)
inline void estimateCovariancesOMP(PointCloud& cloud,
                                   int k = 20,
                                   int num_threads = 4) {
  if (cloud.empty()) return;

  detail::CovarianceSetter setter;
  detail::CovarianceSetter::prepare(cloud);

  search::KdTree tree;
  tree.build(cloud);

  const size_t n = cloud.size();

#pragma omp parallel for num_threads(num_threads) schedule(dynamic, 64)
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate GICP-ready covariances with external KdTree and explicit thread control
inline void estimateCovariancesOMP(PointCloud& cloud,
                                   const search::KdTree& tree,
                                   int k = 20,
                                   int num_threads = 4) {
  if (cloud.empty()) return;

  detail::CovarianceSetter setter;
  detail::CovarianceSetter::prepare(cloud);

  const size_t n = cloud.size();

#pragma omp parallel for num_threads(num_threads) schedule(dynamic, 64)
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate both normals and covariances with explicit thread control
///
/// @param cloud       Point cloud (both channels will be enabled)
/// @param k           Number of neighbors (default: 20)
/// @param num_threads Number of OpenMP threads (default: 4)
/// @param viewpoint   Viewpoint for normal orientation (default: origin)
inline void estimateNormalsCovariancesOMP(PointCloud& cloud,
                                          int k = 20,
                                          int num_threads = 4,
                                          const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return;

  detail::NormalCovarianceSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalCovarianceSetter::prepare(cloud);

  search::KdTree tree;
  tree.build(cloud);

  const size_t n = cloud.size();

#pragma omp parallel for num_threads(num_threads) schedule(dynamic, 64)
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate both normals and covariances with external KdTree and explicit thread control
inline void estimateNormalsCovariancesOMP(PointCloud& cloud,
                                          const search::KdTree& tree,
                                          int k = 20,
                                          int num_threads = 4,
                                          const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return;

  detail::NormalCovarianceSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalCovarianceSetter::prepare(cloud);

  const size_t n = cloud.size();

#pragma omp parallel for num_threads(num_threads) schedule(dynamic, 64)
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

} // namespace geometry
} // namespace nanopcl

#endif // NANOPCL_GEOMETRY_NORMAL_ESTIMATION_OMP_HPP
