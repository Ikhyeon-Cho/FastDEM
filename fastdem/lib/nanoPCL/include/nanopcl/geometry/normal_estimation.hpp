// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Normal and covariance estimation module.
//
// Features:
//   - estimateNormals():           Compute surface normals via PCA
//   - estimateCovariances():       Compute GICP-ready covariances
//   - estimateNormalsCovariances(): Compute both in single pass (optimized)
//
// Parallelization:
//   - Automatically uses OpenMP if compiled with -fopenmp
//   - For explicit thread control, use <nanopcl/geometry/normal_estimation_omp.hpp>

#ifndef NANOPCL_GEOMETRY_NORMAL_ESTIMATION_HPP
#define NANOPCL_GEOMETRY_NORMAL_ESTIMATION_HPP

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/geometry/impl/normal_estimation_impl.hpp"
#include "nanopcl/search/voxel_hash.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace nanopcl {
namespace geometry {

/// @brief Estimate surface normals using PCA
///
/// Computes normal vectors by performing Principal Component Analysis (PCA)
/// on the k-nearest neighbors of each point. The normal is the eigenvector
/// corresponding to the smallest eigenvalue.
///
/// Automatically parallelized with OpenMP if available (-fopenmp).
///
/// Results are stored in cloud.normals() channel.
/// Invalid normals (insufficient neighbors) are set to zero vector.
///
/// @param cloud     Point cloud (normal channel will be enabled/overwritten)
/// @param k         Number of neighbors for KNN search (default: 20)
/// @param viewpoint Viewpoint for normal orientation (default: origin)
inline void estimateNormals(PointCloud& cloud,
                            int k = 20,
                            const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return;

  detail::NormalSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalSetter::prepare(cloud);

  search::KdTree tree;
  tree.build(cloud);

  const size_t n = cloud.size();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate surface normals with external KdTree
///
/// Use this overload when you have a pre-built KdTree to avoid
/// redundant tree construction.
///
/// @param cloud     Point cloud
/// @param tree      Pre-built KdTree
/// @param k         Number of neighbors (default: 20)
/// @param viewpoint Viewpoint for normal orientation (default: origin)
inline void estimateNormals(PointCloud& cloud,
                            const search::KdTree& tree,
                            int k = 20,
                            const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return;

  detail::NormalSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalSetter::prepare(cloud);

  const size_t n = cloud.size();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate GICP-ready covariances using PCA
///
/// Computes regularized covariance matrices suitable for Generalized ICP.
/// The covariance is reconstructed with eigenvalues [epsilon, 1, 1] to
/// enforce planar assumption and improve GICP convergence.
///
/// Automatically parallelized with OpenMP if available.
///
/// Results are stored in cloud.covariances() channel.
/// Invalid covariances (insufficient neighbors) are set to identity matrix.
///
/// @param cloud Point cloud (covariance channel will be enabled/overwritten)
/// @param k     Number of neighbors for KNN search (default: 20)
inline void estimateCovariances(PointCloud& cloud, int k = 20) {
  if (cloud.empty()) return;

  detail::CovarianceSetter setter;
  detail::CovarianceSetter::prepare(cloud);

  search::KdTree tree;
  tree.build(cloud);

  const size_t n = cloud.size();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate GICP-ready covariances with external KdTree
///
/// @param cloud Point cloud
/// @param tree  Pre-built KdTree
/// @param k     Number of neighbors (default: 20)
inline void estimateCovariances(PointCloud& cloud,
                                const search::KdTree& tree,
                                int k = 20) {
  if (cloud.empty()) return;

  detail::CovarianceSetter setter;
  detail::CovarianceSetter::prepare(cloud);

  const size_t n = cloud.size();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate both normals and covariances in a single pass
///
/// More efficient than calling estimateNormals() and estimateCovariances()
/// separately, as it performs PCA only once per point.
///
/// Automatically parallelized with OpenMP if available.
///
/// @param cloud     Point cloud (normal and covariance channels enabled)
/// @param k         Number of neighbors for KNN search (default: 20)
/// @param viewpoint Viewpoint for normal orientation (default: origin)
inline void estimateNormalsCovariances(PointCloud& cloud,
                                       int k = 20,
                                       const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return;

  detail::NormalCovarianceSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalCovarianceSetter::prepare(cloud);

  search::KdTree tree;
  tree.build(cloud);

  const size_t n = cloud.size();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

/// @brief Estimate both normals and covariances with external KdTree
///
/// @param cloud     Point cloud
/// @param tree      Pre-built KdTree
/// @param k         Number of neighbors (default: 20)
/// @param viewpoint Viewpoint for normal orientation (default: origin)
inline void estimateNormalsCovariances(PointCloud& cloud,
                                       const search::KdTree& tree,
                                       int k = 20,
                                       const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return;

  detail::NormalCovarianceSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalCovarianceSetter::prepare(cloud);

  const size_t n = cloud.size();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
  for (size_t i = 0; i < n; ++i) {
    detail::processPoint(cloud, tree, i, k, setter);
  }
}

// =============================================================================
// Radius-based estimation (using VoxelHash for best performance)
// =============================================================================

/// @brief Estimate surface normals using radius search
///
/// Uses VoxelHash internally for optimal radius search performance.
/// Suitable when a fixed physical scale is preferred over adaptive KNN.
///
/// @param cloud     Point cloud (normal channel will be enabled/overwritten)
/// @param radius    Search radius for neighbor finding
/// @param viewpoint Viewpoint for normal orientation (default: origin)
/// @return Number of points with invalid normals (insufficient neighbors)
inline size_t estimateNormals(PointCloud& cloud,
                              float radius,
                              const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return 0;

  detail::NormalSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalSetter::prepare(cloud);

  search::VoxelHash hash(radius);
  hash.build(cloud);

  const size_t n = cloud.size();
  size_t invalid_count = 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64) reduction(+ : invalid_count)
#endif
  for (size_t i = 0; i < n; ++i) {
    invalid_count += detail::processPointRadius(cloud, hash, i, radius, setter);
  }

  return invalid_count;
}

/// @brief Estimate surface normals with external VoxelHash
///
/// @param cloud     Point cloud
/// @param hash      Pre-built VoxelHash
/// @param radius    Search radius
/// @param viewpoint Viewpoint for normal orientation (default: origin)
/// @return Number of invalid points
inline size_t estimateNormals(PointCloud& cloud,
                              const search::VoxelHash& hash,
                              float radius,
                              const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return 0;

  detail::NormalSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalSetter::prepare(cloud);

  const size_t n = cloud.size();
  size_t invalid_count = 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64) reduction(+ : invalid_count)
#endif
  for (size_t i = 0; i < n; ++i) {
    invalid_count += detail::processPointRadius(cloud, hash, i, radius, setter);
  }

  return invalid_count;
}

/// @brief Estimate surface normals using radius search with KdTree
///
/// Use when you already have a KdTree built.
///
/// @param cloud     Point cloud
/// @param tree      Pre-built KdTree
/// @param radius    Search radius
/// @param viewpoint Viewpoint for normal orientation (default: origin)
/// @return Number of invalid points
inline size_t estimateNormals(PointCloud& cloud,
                              const search::KdTree& tree,
                              float radius,
                              const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return 0;

  detail::NormalSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalSetter::prepare(cloud);

  const size_t n = cloud.size();
  size_t invalid_count = 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64) reduction(+ : invalid_count)
#endif
  for (size_t i = 0; i < n; ++i) {
    invalid_count += detail::processPointRadius(cloud, tree, i, radius, setter);
  }

  return invalid_count;
}

/// @brief Estimate GICP-ready covariances using radius search
///
/// @param cloud  Point cloud
/// @param radius Search radius
/// @return Number of invalid points
inline size_t estimateCovariances(PointCloud& cloud, float radius) {
  if (cloud.empty()) return 0;

  detail::CovarianceSetter setter;
  detail::CovarianceSetter::prepare(cloud);

  search::VoxelHash hash(radius);
  hash.build(cloud);

  const size_t n = cloud.size();
  size_t invalid_count = 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64) reduction(+ : invalid_count)
#endif
  for (size_t i = 0; i < n; ++i) {
    invalid_count += detail::processPointRadius(cloud, hash, i, radius, setter);
  }

  return invalid_count;
}

/// @brief Estimate both normals and covariances using radius search
///
/// @param cloud     Point cloud
/// @param radius    Search radius
/// @param viewpoint Viewpoint for normal orientation (default: origin)
/// @return Number of invalid points
inline size_t estimateNormalsCovariances(PointCloud& cloud,
                                         float radius,
                                         const Point& viewpoint = Point::Zero()) {
  if (cloud.empty()) return 0;

  detail::NormalCovarianceSetter setter;
  setter.viewpoint = viewpoint;
  detail::NormalCovarianceSetter::prepare(cloud);

  search::VoxelHash hash(radius);
  hash.build(cloud);

  const size_t n = cloud.size();
  size_t invalid_count = 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64) reduction(+ : invalid_count)
#endif
  for (size_t i = 0; i < n; ++i) {
    invalid_count += detail::processPointRadius(cloud, hash, i, radius, setter);
  }

  return invalid_count;
}

} // namespace geometry
} // namespace nanopcl

#endif // NANOPCL_GEOMETRY_NORMAL_ESTIMATION_HPP
