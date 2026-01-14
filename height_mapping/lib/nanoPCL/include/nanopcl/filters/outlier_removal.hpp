// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_OUTLIER_HPP
#define NANOPCL_FILTERS_OUTLIER_HPP

#include "nanopcl/filters/core.hpp"

namespace npcl {
namespace filters {

// =============================================================================
// Radius Outlier Removal
// =============================================================================

/**
 * @brief Remove outlier points based on neighbor count within radius (copy
 * version)
 *
 * Filters points that have fewer than `min_neighbors` within `radius`.
 * Useful for removing sparse noise or "flying pixels" from LiDAR scans.
 *
 * @note **Performance**: Uses `search::VoxelHash` internally for O(1) radius
 * search. Highly efficient for dense clouds but memory usage scales with
 * spatial extent.
 *
 * @param cloud Input point cloud (unchanged)
 * @param radius Search radius for neighbor counting (meters)
 * @param min_neighbors Minimum number of neighbors required to keep a point
 * @return New point cloud with outliers removed
 *
 * @code
 *   // Keep points that have at least 5 neighbors within 10cm
 *   auto filtered = filters::radiusOutlierRemoval(cloud, 0.1f, 5);
 * @endcode
 */
[[nodiscard]] PointCloud radiusOutlierRemoval(const PointCloud& cloud,
                                              float radius,
                                              size_t min_neighbors);

/**
 * @brief Remove outlier points based on neighbor count within radius (move
 * version)
 */
[[nodiscard]] PointCloud radiusOutlierRemoval(PointCloud&& cloud, float radius,
                                              size_t min_neighbors);

// =============================================================================
// Statistical Outlier Removal
// =============================================================================

/**
 * @brief Remove statistical outliers based on k-nearest neighbor distances
 *
 * Points are filtered based on the distribution of distances to their k-nearest
 * neighbors.
 * 1. For each point, compute mean distance to k nearest neighbors.
 * 2. Compute global mean ($\mu$) and standard deviation ($\sigma$) of these
 * distances.
 * 3. Remove points where mean_distance > $\mu + \alpha \cdot \sigma$.
 *
 * @note **Performance**: Uses `search::KdTree` for KNN search and **OpenMP**
 * for parallel distance calculation. For very large clouds (>1M points),
 * consider downsampling first.
 *
 * Complexity: O(N * k * log N) time, O(N) space.
 *
 * @param cloud Input point cloud (unchanged)
 * @param k Number of nearest neighbors to analyze (typically 8-50)
 * @param std_mul Standard deviation multiplier threshold ($\alpha$)
 * (typically 1.0-2.0) Lower values = more aggressive filtering (removes more
 * points)
 * @return New point cloud with outliers removed
 *
 * @code
 *   // Remove points with mean neighbor distance > μ + 1.0σ
 *   auto clean = filters::statisticalOutlierRemoval(cloud, 30, 1.0f);
 *
 *   // More aggressive filtering
 *   auto aggressive = filters::statisticalOutlierRemoval(cloud, 50, 0.5f);
 * @endcode
 */
[[nodiscard]] PointCloud statisticalOutlierRemoval(const PointCloud& cloud,
                                                   size_t k,
                                                   float std_mul = 1.0f);

/**
 * @brief Remove statistical outliers (move version, reuses input memory)
 */
[[nodiscard]] PointCloud statisticalOutlierRemoval(PointCloud&& cloud, size_t k,
                                                   float std_mul = 1.0f);

}  // namespace filters
}  // namespace npcl

#include "nanopcl/filters/impl/outlier_removal_impl.hpp"

#endif  // NANOPCL_FILTERS_OUTLIER_HPP
