// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_GEOMETRY_NORMAL_ESTIMATION_HPP
#define NANOPCL_GEOMETRY_NORMAL_ESTIMATION_HPP

#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/search/voxel_hash_map.hpp"

namespace nanopcl {
namespace geometry {

/// Configuration for normal estimation
struct NormalEstimationConfig {
  float radius = 0.5f;

  // Minimum neighbors required to compute a valid normal
  // (3 points define a plane, but >3 is better for noise robustness)
  size_t min_neighbors = 5;
};

/**
 * @brief Estimate surface normals using PCA
 *
 * @tparam SearcherT Search object type (must implement radiusSearch)
 * @param cloud Input point cloud
 * @param searcher Initialized search object
 * @param normals [out] Output normals (resized to match cloud)
 * @param config Configuration (radius, etc.)
 */
template <typename SearcherT>
void estimateNormals(const PointCloud& cloud, const SearcherT& searcher,
                     std::vector<Point>& normals,
                     const NormalEstimationConfig& config = {});

/**
 * @brief Convenience overload that builds a temporary searcher
 *
 * @param cloud Input point cloud
 * @param normals [out] Output normals (resized to match cloud)
 * @param config Configuration (radius, etc.)
 */
void estimateNormals(const PointCloud& cloud, std::vector<Point>& normals,
                     const NormalEstimationConfig& config = {});

}  // namespace geometry
}  // namespace nanopcl

// Include implementation
#include "nanopcl/geometry/impl/normal_estimation_impl.hpp"

#endif  // NANOPCL_GEOMETRY_NORMAL_ESTIMATION_HPP
