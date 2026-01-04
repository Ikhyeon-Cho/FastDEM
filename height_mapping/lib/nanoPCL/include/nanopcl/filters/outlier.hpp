// nanopcl/filters/outlier.hpp
// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_OUTLIER_HPP
#define NANOPCL_FILTERS_OUTLIER_HPP

#include "nanopcl/filters/core.hpp"

namespace nanopcl {
namespace filters {

/**
 * @brief Remove outlier points based on neighbor count within radius (copy
 * version)
 *
 * @param cloud Input point cloud (unchanged)
 * @param radius Search radius for neighbor counting
 * @param min_neighbors Minimum number of neighbors required to keep point
 * @return New point cloud with outliers removed
 *
 * @note Points with fewer than min_neighbors within radius are considered
 * outliers. This filter requires spatial indexing (VoxelHashMap) internally.
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

}  // namespace filters
}  // namespace nanopcl

#include "nanopcl/filters/impl/outlier_impl.hpp"

#endif  // NANOPCL_FILTERS_OUTLIER_HPP
