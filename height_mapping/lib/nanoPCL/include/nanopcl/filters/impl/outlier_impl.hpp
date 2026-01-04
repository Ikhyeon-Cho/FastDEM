// nanopcl/filters/impl/outlier_impl.hpp
// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for outlier removal filters.
// Do not include this file directly; include <nanopcl/filters/outlier.hpp>

#ifndef NANOPCL_FILTERS_IMPL_OUTLIER_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_OUTLIER_IMPL_HPP

#include <utility>
#include <vector>

#include "nanopcl/search/voxel_hash_map.hpp"

namespace nanopcl {
namespace filters {

// =============================================================================
// Outlier Removal: radiusOutlierRemoval (copy version)
// =============================================================================
inline PointCloud radiusOutlierRemoval(const PointCloud& cloud, float radius,
                                       size_t min_neighbors) {
  if (cloud.empty()) return PointCloud(cloud.frameId());

  // Build spatial index
  search::VoxelHashMap hash_map(radius);
  hash_map.setInputCloud(cloud);

  // Find inliers
  std::vector<size_t> inlier_indices;
  inlier_indices.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    size_t count = 0;
    hash_map.radiusSearch(cloud[i], radius, [&](uint32_t idx, const Point&) {
      if (idx != i) ++count;
    });
    if (count >= min_neighbors) {
      inlier_indices.push_back(i);
    }
  }

  return cloud[inlier_indices];
}

// =============================================================================
// Outlier Removal: radiusOutlierRemoval (move version)
// =============================================================================
inline PointCloud radiusOutlierRemoval(PointCloud&& cloud, float radius,
                                       size_t min_neighbors) {
  if (cloud.empty()) return std::move(cloud);

  // Build spatial index
  search::VoxelHashMap hash_map(radius);
  hash_map.setInputCloud(cloud);

  // Find inliers using a bitmask for in-place filtering
  std::vector<bool> keep(cloud.size(), false);

  for (size_t i = 0; i < cloud.size(); ++i) {
    size_t count = 0;
    hash_map.radiusSearch(cloud[i], radius, [&](uint32_t idx, const Point&) {
      if (idx != i) ++count;
    });
    if (count >= min_neighbors) {
      keep[i] = true;
    }
  }

  // In-place compaction using index tracking
  size_t idx = 0;
  detail::filterByPointInPlace(
      cloud, [&keep, &idx](const Point&) { return keep[idx++]; });

  return std::move(cloud);
}

}  // namespace filters
}  // namespace nanopcl

#endif  // NANOPCL_FILTERS_IMPL_OUTLIER_IMPL_HPP
