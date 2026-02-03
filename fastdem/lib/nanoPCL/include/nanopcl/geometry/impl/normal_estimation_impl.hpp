// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Core implementation for normal/covariance estimation.
// This file contains pure algorithm logic with NO parallelization dependencies.

#ifndef NANOPCL_GEOMETRY_IMPL_NORMAL_ESTIMATION_IMPL_HPP
#define NANOPCL_GEOMETRY_IMPL_NORMAL_ESTIMATION_IMPL_HPP

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/geometry/impl/pca.hpp"
#include "nanopcl/geometry/impl/setters.hpp"
#include "nanopcl/search/kdtree.hpp"

namespace nanopcl {
namespace geometry {
namespace detail {

/// @brief Minimum neighbors required for valid PCA
constexpr size_t MIN_NEIGHBORS = 3;

/// @brief Process a single point (thread-safe, pure algorithm)
///
/// Performs KNN search, PCA computation, and result storage via Setter.
/// This function is designed to be called from any loop (serial or parallel).
///
/// @tparam Setter Policy class (NormalSetter, CovarianceSetter, etc.)
/// @param cloud Point cloud (must have channels prepared)
/// @param tree KdTree for neighbor search
/// @param i Point index to process
/// @param k Number of neighbors
/// @param setter Setter instance with configuration (e.g., viewpoint)
template <typename Setter>
inline void processPoint(PointCloud& cloud,
                         const search::KdTree& tree,
                         size_t i,
                         int k,
                         const Setter& setter) {
  // Get query point
  const Point query = cloud.point(i);

  // KNN search
  auto neighbors = tree.knn(query, static_cast<size_t>(k));

  // Check minimum neighbors
  if (neighbors.size() < MIN_NEIGHBORS) {
    Setter::setInvalid(cloud, i);
    return;
  }

  // Extract indices from search results
  std::vector<uint32_t> indices;
  indices.reserve(neighbors.size());
  for (const auto& nb : neighbors) {
    indices.push_back(nb.index);
  }

  // Compute PCA
  PCAResult pca = computePCA(cloud, indices);

  if (!pca.valid) {
    Setter::setInvalid(cloud, i);
    return;
  }

  // Apply setter policy
  setter.set(cloud, i, pca.eigenvectors, pca.eigenvalues);
}

/// @brief Process a single point using radius search (thread-safe)
///
/// @tparam Searcher Search structure with radius() method (KdTree or VoxelHash)
/// @tparam Setter Policy class (NormalSetter, CovarianceSetter, etc.)
/// @param cloud Point cloud (must have channels prepared)
/// @param searcher Search structure for radius search
/// @param i Point index to process
/// @param radius Search radius
/// @param setter Setter instance with configuration
/// @return 1 if invalid (insufficient neighbors), 0 otherwise
template <typename Searcher, typename Setter>
inline size_t processPointRadius(PointCloud& cloud,
                                 const Searcher& searcher,
                                 size_t i,
                                 float radius,
                                 const Setter& setter) {
  const Point query = cloud.point(i);

  // Radius search
  std::vector<uint32_t> indices = searcher.radius(query, radius);

  if (indices.size() < MIN_NEIGHBORS) {
    Setter::setInvalid(cloud, i);
    return 1;
  }

  PCAResult pca = computePCA(cloud, indices);

  if (!pca.valid) {
    Setter::setInvalid(cloud, i);
    return 1;
  }

  setter.set(cloud, i, pca.eigenvectors, pca.eigenvalues);
  return 0;
}

} // namespace detail
} // namespace geometry
} // namespace nanopcl

#endif // NANOPCL_GEOMETRY_IMPL_NORMAL_ESTIMATION_IMPL_HPP
