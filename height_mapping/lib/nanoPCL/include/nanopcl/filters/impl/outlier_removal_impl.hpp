// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for outlier removal filters.
// Do not include this file directly; include <nanopcl/filters/outlier.hpp>

#ifndef NANOPCL_FILTERS_IMPL_OUTLIER_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_OUTLIER_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "nanopcl/search/kdtree.hpp"
#include "nanopcl/search/voxel_hash.hpp"

namespace npcl {
namespace filters {

// =============================================================================
// Outlier Removal: radiusOutlierRemoval (copy version)
// =============================================================================
inline PointCloud radiusOutlierRemoval(const PointCloud& cloud, float radius,
                                       size_t min_neighbors) {
  if (cloud.empty()) return PointCloud(cloud.frameId());

  // Build spatial index
  search::VoxelHash voxel_hash(radius);
  voxel_hash.build(cloud);

  // Find inliers
  std::vector<size_t> inlier_indices;
  inlier_indices.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    size_t count = 0;
    voxel_hash.radius(cloud[i], radius, [&](uint32_t idx, const Point&, float) {
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
  search::VoxelHash voxel_hash(radius);
  voxel_hash.build(cloud);

  // Find inliers using a bitmask for in-place filtering
  std::vector<bool> keep(cloud.size(), false);

  for (size_t i = 0; i < cloud.size(); ++i) {
    size_t count = 0;
    voxel_hash.radius(cloud[i], radius, [&](uint32_t idx, const Point&, float) {
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

// =============================================================================
// Outlier Removal: statisticalOutlierRemoval (copy version)
// =============================================================================
inline PointCloud statisticalOutlierRemoval(const PointCloud& cloud, size_t k,
                                            float std_mul) {
  if (cloud.empty() || k == 0) {
    PointCloud result(cloud.frameId());
    result.setTimestamp(cloud.timestamp());
    return result;
  }

  const size_t n = cloud.size();

  // Handle edge case: if k >= n, all points are neighbors
  const size_t effective_k = std::min(k, n - 1);
  if (effective_k == 0) {
    return cloud;  // Only one point, nothing to filter
  }

  // Step 1: Build KdTree
  search::KdTree kdtree;
  kdtree.build(cloud);

  // Step 2: Compute mean distance to k neighbors for each point
  std::vector<float> mean_distances(n);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 256)
#endif
  for (size_t i = 0; i < n; ++i) {
    // k+1 because knn may include the query point itself
    auto neighbors = kdtree.knn(cloud[i], effective_k + 1);

    float sum = 0.0f;
    size_t count = 0;
    for (const auto& nb : neighbors) {
      if (nb.index != static_cast<uint32_t>(i)) {
        sum += std::sqrt(nb.dist_sq);
        ++count;
        if (count >= effective_k) break;
      }
    }
    mean_distances[i] = (count > 0) ? sum / static_cast<float>(count) : 0.0f;
  }

  // Step 3: Compute global mean and standard deviation
  double sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    sum += mean_distances[i];
  }
  const double global_mean = sum / static_cast<double>(n);

  double sum_sq_diff = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double diff = mean_distances[i] - global_mean;
    sum_sq_diff += diff * diff;
  }
  const float global_std =
      static_cast<float>(std::sqrt(sum_sq_diff / static_cast<double>(n)));

  // Step 4: Filter by threshold
  const float threshold =
      static_cast<float>(global_mean) + std_mul * global_std;

  std::vector<size_t> inlier_indices;
  inlier_indices.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    if (mean_distances[i] <= threshold) {
      inlier_indices.push_back(i);
    }
  }

  return cloud[inlier_indices];
}

// =============================================================================
// Outlier Removal: statisticalOutlierRemoval (move version)
// =============================================================================
inline PointCloud statisticalOutlierRemoval(PointCloud&& cloud, size_t k,
                                            float std_mul) {
  if (cloud.empty() || k == 0) {
    return std::move(cloud);
  }

  const size_t n = cloud.size();

  const size_t effective_k = std::min(k, n - 1);
  if (effective_k == 0) {
    return std::move(cloud);
  }

  // Step 1: Build KdTree
  search::KdTree kdtree;
  kdtree.build(cloud);

  // Step 2: Compute mean distance to k neighbors for each point
  std::vector<float> mean_distances(n);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 256)
#endif
  for (size_t i = 0; i < n; ++i) {
    auto neighbors = kdtree.knn(cloud[i], effective_k + 1);

    float sum = 0.0f;
    size_t count = 0;
    for (const auto& nb : neighbors) {
      if (nb.index != static_cast<uint32_t>(i)) {
        sum += std::sqrt(nb.dist_sq);
        ++count;
        if (count >= effective_k) break;
      }
    }
    mean_distances[i] = (count > 0) ? sum / static_cast<float>(count) : 0.0f;
  }

  // Step 3: Compute global statistics
  double sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    sum += mean_distances[i];
  }
  const double global_mean = sum / static_cast<double>(n);

  double sum_sq_diff = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double diff = mean_distances[i] - global_mean;
    sum_sq_diff += diff * diff;
  }
  const float global_std =
      static_cast<float>(std::sqrt(sum_sq_diff / static_cast<double>(n)));

  const float threshold =
      static_cast<float>(global_mean) + std_mul * global_std;

  // Step 4: In-place filtering with bitmask
  std::vector<bool> keep(n);
  for (size_t i = 0; i < n; ++i) {
    keep[i] = (mean_distances[i] <= threshold);
  }

  size_t idx = 0;
  detail::filterByPointInPlace(
      cloud, [&keep, &idx](const Point&) { return keep[idx++]; });

  return std::move(cloud);
}

}  // namespace filters
}  // namespace npcl

#endif  // NANOPCL_FILTERS_IMPL_OUTLIER_IMPL_HPP
