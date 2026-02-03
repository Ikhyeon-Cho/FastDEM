// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_OUTLIER_REMOVAL_HPP
#define NANOPCL_FILTERS_OUTLIER_REMOVAL_HPP

#include "nanopcl/filters/core.hpp"

namespace nanopcl {
namespace filters {

// Radius Outlier Removal (VoxelHash based, O(1) radius search)
[[nodiscard]] PointCloud radiusOutlierRemoval(const PointCloud& cloud,
                                              float radius,
                                              size_t min_neighbors);
[[nodiscard]] PointCloud radiusOutlierRemoval(PointCloud&& cloud, float radius, size_t min_neighbors);

// Statistical Outlier Removal (KdTree based KNN)
[[nodiscard]] PointCloud statisticalOutlierRemoval(const PointCloud& cloud,
                                                   size_t k,
                                                   float std_mul = 1.0f);
[[nodiscard]] PointCloud statisticalOutlierRemoval(PointCloud&& cloud, size_t k, float std_mul = 1.0f);

} // namespace filters
} // namespace nanopcl

#include "nanopcl/filters/impl/outlier_removal_impl.hpp"

#endif // NANOPCL_FILTERS_OUTLIER_REMOVAL_HPP
