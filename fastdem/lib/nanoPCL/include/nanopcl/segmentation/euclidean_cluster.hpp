// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Euclidean Clustering
//
// Groups nearby points into clusters using BFS-based region growing.
// Optimized for obstacle detection after ground plane removal.
//
// Example usage:
//   // After ground removal
//   auto ground = segmentation::segmentPlane(cloud, 0.1f);
//   auto obstacles = cloud[ground.outliers(cloud.size())];
//
//   // Cluster obstacles
//   auto clusters = segmentation::euclideanCluster(obstacles, {.tolerance =
//   0.5f});
//
//   // Process each cluster
//   for (size_t i = 0; i < clusters.numClusters(); ++i) {
//     PointCloud obj = clusters.extract(obstacles, i);
//     // ... compute bounding box, track object, etc.
//   }

#ifndef NANOPCL_SEGMENTATION_EUCLIDEAN_CLUSTER_HPP
#define NANOPCL_SEGMENTATION_EUCLIDEAN_CLUSTER_HPP

#include <cstdint>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/core/span.hpp"

namespace nanopcl {
namespace segmentation {

// =============================================================================
// Cluster Configuration
// =============================================================================

/**
 * @brief Configuration for Euclidean clustering
 */
struct ClusterConfig {
  float tolerance = 0.5f;   ///< Distance threshold for neighbors (meters)
  size_t min_size = 10;     ///< Minimum points per cluster
  size_t max_size = 100000; ///< Maximum points per cluster
};

// =============================================================================
// Cluster Result (CSR Format)
// =============================================================================

/**
 * @brief Result of Euclidean clustering in CSR (Compressed Sparse Row) format
 *
 * Stores cluster assignments efficiently using two flat arrays to minimize 
 * memory fragmentation and improve cache locality.
 *
 * - `indices`: A single vector containing all clustered point indices.
 * - `offsets`: Pointers to the start of each cluster within the `indices` vector.
 *
 * @code
 *   auto clusters = segmentation::euclideanCluster(cloud, config);
 *   for (size_t i = 0; i < clusters.numClusters(); ++i) {
 *     // Get a view over indices of cluster i (zero copy)
 *     auto idx_view = clusters.clusterIndices(i);
 *     for (uint32_t idx : idx_view) {
 *       const auto& pt = cloud[idx];
 *     }
 *   }
 * @endcode
 */
struct ClusterResult {
  std::vector<uint32_t> indices; ///< All cluster point indices (flat)
  std::vector<uint32_t> offsets; ///< Cluster boundaries [0, n0, n0+n1, ...]

  // ===========================================================================
  // Cluster Information
  // ===========================================================================

  /// Number of clusters found
  [[nodiscard]] size_t numClusters() const noexcept {
    return offsets.empty() ? 0 : offsets.size() - 1;
  }

  /// Number of points in cluster i
  [[nodiscard]] size_t clusterSize(size_t i) const noexcept {
    return offsets[i + 1] - offsets[i];
  }

  /// Total number of clustered points (excludes noise)
  [[nodiscard]] size_t totalClusteredPoints() const noexcept {
    return indices.size();
  }

  /// Check if any clusters were found
  [[nodiscard]] bool empty() const noexcept { return numClusters() == 0; }

  // ===========================================================================
  // Cluster Access
  // ===========================================================================

  /**
   * @brief Get indices of points in cluster i
   * @param i Cluster index (0-based)
   * @return Span view over point indices (no copy)
   *
   * @code
   *   for (uint32_t idx : result.clusterIndices(0)) {
   *     process(cloud[idx]);
   *   }
   * @endcode
   */
  [[nodiscard]] Span<const uint32_t> clusterIndices(size_t i) const {
    return Span<const uint32_t>(indices.data() + offsets[i],
                                offsets[i + 1] - offsets[i]);
  }

  /**
   * @brief Extract cluster as a new PointCloud
   * @param cloud Source point cloud
   * @param i Cluster index (0-based)
   * @return New PointCloud containing only cluster points with all attributes
   */
  [[nodiscard]] PointCloud extract(const PointCloud& cloud, size_t i) const;

  // ===========================================================================
  // Noise Handling
  // ===========================================================================

  /**
   * @brief Get indices of noise points (not in any cluster)
   * @param total_points Total number of points in the original cloud
   * @return Vector of noise point indices
   */
  [[nodiscard]] std::vector<uint32_t> noiseIndices(size_t total_points) const;

  // ===========================================================================
  // Statistics
  // ===========================================================================

  /**
   * @brief Get sizes of all clusters
   * @return Vector of cluster sizes (sorted by cluster index)
   */
  [[nodiscard]] std::vector<size_t> clusterSizes() const;
};

// =============================================================================
// Euclidean Clustering API
// =============================================================================

/**
 * @brief Cluster points using Euclidean distance
 *
 * Uses BFS-based region growing with spatial hashing for fast neighbor queries.
 * Points within 'tolerance' distance are grouped into the same cluster.
 *
 * Algorithm:
 * 1. Build spatial hash index (O(N))
 * 2. For each unvisited point, start BFS to find connected component
 * 3. Accept cluster if size is within [min_size, max_size]
 *
 * Complexity: O(N) average case with spatial hashing
 *
 * @param cloud Input point cloud
 * @param config Clustering configuration
 * @return ClusterResult with indices in CSR format
 */
[[nodiscard]] ClusterResult euclideanCluster(const PointCloud& cloud,
                                             const ClusterConfig& config = {});

/// @brief Convenience overload with direct parameters
[[nodiscard]] inline ClusterResult euclideanCluster(const PointCloud& cloud,
                                                    float tolerance,
                                                    size_t min_size = 10,
                                                    size_t max_size = 100000) {
  return euclideanCluster(cloud, ClusterConfig{tolerance, min_size, max_size});
}

/**
 * @brief Cluster a subset of points using Euclidean distance
 *
 * Useful for clustering after ground removal:
 * @code
 *   auto ground = segmentPlane(cloud, 0.1f);
 *   auto clusters = euclideanCluster(cloud, ground.outliers(cloud.size()));
 * @endcode
 *
 * @param cloud Input point cloud (full cloud)
 * @param subset_indices Indices of points to cluster
 * @param config Clustering configuration
 * @return ClusterResult with indices referencing the ORIGINAL cloud
 *
 * @note Returned indices reference the original cloud, not the subset.
 */
[[nodiscard]] ClusterResult euclideanCluster(
    const PointCloud& cloud, const std::vector<uint32_t>& subset_indices, const ClusterConfig& config = {});

/// @brief Convenience overload with direct parameters (subset version)
[[nodiscard]] inline ClusterResult euclideanCluster(
    const PointCloud& cloud,
    const std::vector<uint32_t>& subset_indices,
    float tolerance,
    size_t min_size = 10,
    size_t max_size = 100000) {
  return euclideanCluster(cloud, subset_indices, ClusterConfig{tolerance, min_size, max_size});
}

// =============================================================================
// Label Channel Integration
// =============================================================================

/**
 * @brief Apply cluster IDs to the Label channel
 *
 * Marks each point's instanceId with its cluster ID (1-based).
 * Noise points (not in any cluster) get instanceId = 0.
 *
 * @param cloud Point cloud to modify (Label channel will be enabled)
 * @param result Clustering result
 * @param semantic_class Optional semantic class to set (default: 0)
 *
 * @note Existing Label values will be overwritten.
 *
 * @code
 *   auto clusters = euclideanCluster(cloud, config);
 *   applyClusterLabels(cloud, clusters);
 *
 *   // Now filter by cluster
 *   auto cluster_0 = filters::filter(cloud, [](auto p) {
 *     return p.label().instanceId() == 1;
 *   });
 * @endcode
 */
void applyClusterLabels(PointCloud& cloud, const ClusterResult& result, uint16_t semantic_class = 0);

} // namespace segmentation
} // namespace nanopcl

#include "nanopcl/segmentation/impl/euclidean_cluster_impl.hpp"

#endif // NANOPCL_SEGMENTATION_EUCLIDEAN_CLUSTER_HPP
