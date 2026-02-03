// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Euclidean clustering implementation.
// Do not include this file directly; include
// <nanopcl/segmentation/euclidean_cluster.hpp>

#ifndef NANOPCL_SEGMENTATION_IMPL_EUCLIDEAN_CLUSTER_IMPL_HPP
#define NANOPCL_SEGMENTATION_IMPL_EUCLIDEAN_CLUSTER_IMPL_HPP

#include <algorithm>
#include <queue>
#include <vector>

#include "nanopcl/search/voxel_hash.hpp"

namespace nanopcl {
namespace segmentation {

// =============================================================================
// ClusterResult Implementation
// =============================================================================

inline PointCloud ClusterResult::extract(const PointCloud& cloud,
                                         size_t i) const {
  // Convert to size_t indices for operator[]
  std::vector<size_t> size_t_indices;
  size_t_indices.reserve(clusterSize(i));

  for (uint32_t idx : clusterIndices(i)) {
    size_t_indices.push_back(static_cast<size_t>(idx));
  }

  return cloud.extract(size_t_indices);
}

inline std::vector<uint32_t> ClusterResult::noiseIndices(
    size_t total_points) const {
  // Create visited mask
  std::vector<bool> in_cluster(total_points, false);
  for (uint32_t idx : indices) {
    in_cluster[idx] = true;
  }

  // Collect noise points
  std::vector<uint32_t> noise;
  noise.reserve(total_points - indices.size());

  for (size_t i = 0; i < total_points; ++i) {
    if (!in_cluster[i]) {
      noise.push_back(static_cast<uint32_t>(i));
    }
  }

  return noise;
}

inline std::vector<size_t> ClusterResult::clusterSizes() const {
  std::vector<size_t> sizes;
  sizes.reserve(numClusters());

  for (size_t i = 0; i < numClusters(); ++i) {
    sizes.push_back(clusterSize(i));
  }

  return sizes;
}

// =============================================================================
// BFS Clustering Core Algorithm
// =============================================================================

namespace detail {

/**
 * @brief BFS-based region growing for a single cluster
 *
 * Grows a cluster by exploring all connected points within the tolerance
 * distance. The entire connected component is discovered (PCL-style behavior).
 *
 * @param cloud Point cloud (or subset view)
 * @param search Spatial index for radius queries
 * @param seed_idx Starting point index
 * @param tolerance Distance threshold
 * @param visited Visited flags (modified in-place)
 * @param cluster_indices Output: indices belonging to this cluster
 */
template <typename SearchIndex>
inline void growCluster(const PointCloud& cloud, const SearchIndex& search, uint32_t seed_idx, float tolerance, std::vector<bool>& visited, std::vector<uint32_t>& cluster_indices) {
  cluster_indices.clear();
  std::queue<uint32_t> queue;

  // Start with seed point
  queue.push(seed_idx);
  visited[seed_idx] = true;

  std::vector<uint32_t> neighbors;
  neighbors.reserve(64);

  // Grow cluster completely (PCL-style: discover full connected component)
  while (!queue.empty()) {
    uint32_t current = queue.front();
    queue.pop();
    cluster_indices.push_back(current);

    // Find neighbors within tolerance
    neighbors.clear();
    search.radius(cloud.point(current), tolerance, neighbors);

    for (uint32_t neighbor_idx : neighbors) {
      if (!visited[neighbor_idx]) {
        visited[neighbor_idx] = true;
        queue.push(neighbor_idx);
      }
    }
  }
}

/**
 * @brief Core clustering implementation
 */
inline ClusterResult clusterCore(const PointCloud& cloud,
                                 const ClusterConfig& config) {
  ClusterResult result;

  if (cloud.empty()) {
    result.offsets.push_back(0);
    return result;
  }

  const size_t n = cloud.size();

  // Build spatial index
  // Use tolerance as voxel resolution for efficient neighbor queries
  search::VoxelHash search(config.tolerance);
  search.build(cloud);

  // Clustering state
  std::vector<bool> visited(n, false);
  std::vector<uint32_t> cluster_indices;
  cluster_indices.reserve(std::min(config.max_size, n));

  // Temporary storage for all clusters
  std::vector<std::vector<uint32_t>> clusters;

  // Process each unvisited point
  for (uint32_t i = 0; i < n; ++i) {
    if (visited[i])
      continue;

    // Grow cluster from this seed (discovers full connected component)
    growCluster(cloud, search, i, config.tolerance, visited, cluster_indices);

    // Accept cluster only if within size bounds (PCL-style)
    // Oversized clusters are rejected entirely
    if (cluster_indices.size() >= config.min_size &&
        cluster_indices.size() <= config.max_size) {
      clusters.push_back(std::move(cluster_indices));
      cluster_indices = std::vector<uint32_t>();
      cluster_indices.reserve(n);
    }
    // Note: rejected clusters' points remain visited, preventing
    // them from forming other clusters (intentional PCL behavior)
  }

  // Sort clusters by size (largest first)
  std::sort(clusters.begin(), clusters.end(), [](const auto& a, const auto& b) { return a.size() > b.size(); });

  // Convert to CSR format
  size_t total_points = 0;
  for (const auto& cluster : clusters) {
    total_points += cluster.size();
  }

  result.indices.reserve(total_points);
  result.offsets.reserve(clusters.size() + 1);
  result.offsets.push_back(0);

  for (const auto& cluster : clusters) {
    result.indices.insert(result.indices.end(), cluster.begin(), cluster.end());
    result.offsets.push_back(static_cast<uint32_t>(result.indices.size()));
  }

  return result;
}

} // namespace detail

// =============================================================================
// Public API Implementation
// =============================================================================

inline ClusterResult euclideanCluster(const PointCloud& cloud,
                                      const ClusterConfig& config) {
  return detail::clusterCore(cloud, config);
}

inline ClusterResult euclideanCluster(
    const PointCloud& cloud, const std::vector<uint32_t>& subset_indices, const ClusterConfig& config) {
  if (subset_indices.empty()) {
    ClusterResult result;
    result.offsets.push_back(0);
    return result;
  }

  // Create subset cloud for clustering
  std::vector<size_t> size_t_indices(subset_indices.begin(),
                                     subset_indices.end());
  PointCloud subset = cloud.extract(size_t_indices);

  // Cluster the subset
  ClusterResult subset_result = detail::clusterCore(subset, config);

  // Remap indices back to original cloud indices
  ClusterResult result;
  result.indices.reserve(subset_result.indices.size());
  result.offsets = std::move(subset_result.offsets);

  for (uint32_t subset_idx : subset_result.indices) {
    result.indices.push_back(subset_indices[subset_idx]);
  }

  return result;
}

inline void applyClusterLabels(PointCloud& cloud, const ClusterResult& result, uint16_t semantic_class) {
  // Enable label channel
  cloud.useLabel();

  // Initialize all labels to noise (instanceId = 0)
  // Label format: upper 16 bits = semantic_class, lower 16 bits = instance_id
  uint32_t noise_label = (static_cast<uint32_t>(semantic_class) << 16) | 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    cloud.labels()[i] = Label(noise_label);
  }

  // Mark cluster points (1-based instance IDs)
  for (size_t cluster_id = 0; cluster_id < result.numClusters(); ++cluster_id) {
    uint16_t instance_id = static_cast<uint16_t>(cluster_id + 1);
    uint32_t label_val = (static_cast<uint32_t>(semantic_class) << 16) | instance_id;

    for (uint32_t idx : result.clusterIndices(cluster_id)) {
      cloud.labels()[idx] = Label(label_val);
    }
  }
}

} // namespace segmentation
} // namespace nanopcl

#endif // NANOPCL_SEGMENTATION_IMPL_EUCLIDEAN_CLUSTER_IMPL_HPP
