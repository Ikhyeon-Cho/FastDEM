// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_IMPL_OUTLIER_REMOVAL_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_OUTLIER_REMOVAL_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include "nanopcl/search/kdtree.hpp"
#include "nanopcl/search/voxel_hash.hpp"

namespace nanopcl {
namespace filters {

// =============================================================================
// Radius Outlier Removal
// =============================================================================

inline PointCloud radiusOutlierRemoval(const PointCloud& cloud,
                                       float radius,
                                       size_t min_neighbors) {
  if (cloud.empty())
    return PointCloud();

  search::VoxelHash voxel_hash(radius);
  voxel_hash.build(cloud);

  std::vector<size_t> inliers;
  inliers.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    size_t count = 0;
    Eigen::Vector3f query = cloud.point(i);
    voxel_hash.radius(query, radius, [&](uint32_t idx, const auto&, float) {
      if (idx != i)
        ++count;
    });
    if (count >= min_neighbors)
      inliers.push_back(i);
  }

  PointCloud result = cloud.extract(inliers);
  result.setFrameId(cloud.frameId());
  result.setTimestamp(cloud.timestamp());
  return result;
}

inline PointCloud radiusOutlierRemoval(PointCloud&& cloud,
                                       float radius,
                                       size_t min_neighbors) {
  if (cloud.empty())
    return std::move(cloud);

  search::VoxelHash voxel_hash(radius);
  voxel_hash.build(cloud);

  // Count neighbors for each point
  const size_t n = cloud.size();
  std::vector<size_t> neighbor_counts(n, 0);

  for (size_t i = 0; i < n; ++i) {
    Eigen::Vector3f query = cloud.point(i);
    voxel_hash.radius(query, radius, [&](uint32_t idx, const auto&, float) {
      if (idx != i)
        ++neighbor_counts[i];
    });
  }

  // In-place filtering using neighbor counts
  detail::filterInPlace(cloud, [&](size_t i) {
    return neighbor_counts[i] >= min_neighbors;
  });

  return std::move(cloud);
}

// =============================================================================
// Statistical Outlier Removal
// =============================================================================

inline PointCloud statisticalOutlierRemoval(const PointCloud& cloud,
                                            size_t k,
                                            float std_mul) {
  if (cloud.empty() || k == 0)
    return PointCloud();

  const size_t n = cloud.size();
  const size_t effective_k = std::min(k, n - 1);
  if (effective_k == 0)
    return cloud.extract({}); // Only one point

  search::KdTree kdtree;
  kdtree.build(cloud);

  std::vector<float> mean_distances(n);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 256)
#endif
  for (size_t i = 0; i < n; ++i) {
    Eigen::Vector3f query = cloud.point(i);
    auto neighbors = kdtree.knn(query, effective_k + 1);

    float sum = 0.0f;
    size_t count = 0;
    for (const auto& nb : neighbors) {
      if (nb.index != static_cast<uint32_t>(i)) {
        sum += std::sqrt(nb.dist_sq);
        if (++count >= effective_k)
          break;
      }
    }
    mean_distances[i] = (count > 0) ? sum / static_cast<float>(count) : 0.0f;
  }

  double sum = 0.0;
  for (size_t i = 0; i < n; ++i)
    sum += mean_distances[i];
  const double global_mean = sum / static_cast<double>(n);

  double sum_sq_diff = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double diff = mean_distances[i] - global_mean;
    sum_sq_diff += diff * diff;
  }
  const float global_std = static_cast<float>(std::sqrt(sum_sq_diff / static_cast<double>(n)));
  const float threshold = static_cast<float>(global_mean) + std_mul * global_std;

  std::vector<size_t> inliers;
  inliers.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    if (mean_distances[i] <= threshold)
      inliers.push_back(i);
  }

  PointCloud result = cloud.extract(inliers);
  result.setFrameId(cloud.frameId());
  result.setTimestamp(cloud.timestamp());
  return result;
}

inline PointCloud statisticalOutlierRemoval(PointCloud&& cloud,
                                            size_t k,
                                            float std_mul) {
  if (cloud.empty() || k == 0)
    return std::move(cloud);

  const size_t n = cloud.size();
  const size_t effective_k = std::min(k, n - 1);
  if (effective_k == 0)
    return std::move(cloud);

  search::KdTree kdtree;
  kdtree.build(cloud);

  std::vector<float> mean_distances(n);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 256)
#endif
  for (size_t i = 0; i < n; ++i) {
    Eigen::Vector3f query = cloud.point(i);
    auto neighbors = kdtree.knn(query, effective_k + 1);

    float sum = 0.0f;
    size_t count = 0;
    for (const auto& nb : neighbors) {
      if (nb.index != static_cast<uint32_t>(i)) {
        sum += std::sqrt(nb.dist_sq);
        if (++count >= effective_k)
          break;
      }
    }
    mean_distances[i] = (count > 0) ? sum / static_cast<float>(count) : 0.0f;
  }

  double sum = 0.0;
  for (size_t i = 0; i < n; ++i)
    sum += mean_distances[i];
  const double global_mean = sum / static_cast<double>(n);

  double sum_sq_diff = 0.0;
  for (size_t i = 0; i < n; ++i) {
    double diff = mean_distances[i] - global_mean;
    sum_sq_diff += diff * diff;
  }
  const float global_std = static_cast<float>(std::sqrt(sum_sq_diff / static_cast<double>(n)));
  const float threshold = static_cast<float>(global_mean) + std_mul * global_std;

  // In-place filtering using mean_distances directly (no extra keep array)
  detail::filterInPlace(cloud, [&](size_t i) {
    return mean_distances[i] <= threshold;
  });

  return std::move(cloud);
}

} // namespace filters
} // namespace nanopcl

#endif // NANOPCL_FILTERS_IMPL_OUTLIER_REMOVAL_IMPL_HPP
