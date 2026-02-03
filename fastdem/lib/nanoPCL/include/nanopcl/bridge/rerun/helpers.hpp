// nanoPCL - High-level visualization helpers for common use cases
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_VISUALIZATION_RERUN_HELPERS_HPP
#define NANOPCL_VISUALIZATION_RERUN_HELPERS_HPP

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/segmentation/euclidean_cluster.hpp"

#include <string>
#include <vector>

#ifdef NANOPCL_USE_RERUN

#include <rerun.hpp>

#include "adapters.hpp"
#include "stream.hpp"

namespace nanopcl::rr {

/// Predefined color palette for cluster visualization
namespace palette {
inline const std::vector<rerun::Color> distinct = {
    {230, 25, 75},   // Red
    {60, 180, 75},   // Green
    {0, 130, 200},   // Blue
    {255, 225, 25},  // Yellow
    {145, 30, 180},  // Purple
    {70, 240, 240},  // Cyan
    {245, 130, 48},  // Orange
    {240, 50, 230},  // Magenta
    {188, 246, 12},  // Lime
    {250, 190, 212}, // Pink
    {0, 128, 128},   // Teal
    {154, 99, 36},   // Brown
    {128, 0, 0},     // Maroon
    {128, 128, 0},   // Olive
    {0, 0, 128},     // Navy
};
} // namespace palette

/// Visualize clustering result with automatic color assignment
/// @param path Entity path in Rerun
/// @param cloud Source point cloud
/// @param clusters Vector of index vectors, each representing a cluster
/// @param point_radius Point display radius
inline void showClusters(const std::string& path, const PointCloud& cloud,
                         const std::vector<std::vector<size_t>>& clusters,
                         float point_radius = 0.02f) {
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].empty()) {
      continue;
    }

    PointCloud cluster_cloud = cloud.extract(clusters[i]);
    const auto& color = palette::distinct[i % palette::distinct.size()];

    stream().log(path + "/cluster_" + std::to_string(i),
                 adapters::toPoints3DWithColor(cluster_cloud, color)
                     .with_radii({point_radius}));
  }
}

/// Visualize ClusterResult from segmentation module
/// @param path Entity path in Rerun
/// @param cloud Source point cloud
/// @param result ClusterResult from euclideanCluster()
/// @param point_radius Point display radius
inline void showClusters(const std::string& path, const PointCloud& cloud,
                         const segmentation::ClusterResult& result,
                         float point_radius = 0.02f) {
  for (size_t i = 0; i < result.numClusters(); ++i) {
    auto indices_span = result.clusterIndices(i);
    std::vector<size_t> indices(indices_span.begin(), indices_span.end());

    PointCloud cluster_cloud = cloud.extract(indices);
    const auto& color = palette::distinct[i % palette::distinct.size()];

    stream().log(path + "/cluster_" + std::to_string(i),
                 adapters::toPoints3DWithColor(cluster_cloud, color)
                     .with_radii({point_radius}));
  }
}

/// Visualize RANSAC plane segmentation result
/// @param path Entity path in Rerun
/// @param cloud Source point cloud
/// @param inlier_indices Indices of points belonging to the plane
/// @param point_radius Point display radius
template <typename IndexType>
inline void showPlaneSegmentation(const std::string& path,
                                  const PointCloud& cloud,
                                  const std::vector<IndexType>& inlier_indices,
                                  float point_radius = 0.02f) {
  // Create inlier set for fast lookup
  std::vector<bool> is_inlier(cloud.size(), false);
  for (size_t idx : inlier_indices) {
    if (idx < cloud.size()) {
      is_inlier[idx] = true;
    }
  }

  // Extract inliers and outliers
  std::vector<size_t> outlier_indices;
  outlier_indices.reserve(cloud.size() - inlier_indices.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (!is_inlier[i]) {
      outlier_indices.push_back(i);
    }
  }

  // Convert to size_t for extract()
  std::vector<size_t> inliers_size_t(inlier_indices.begin(), inlier_indices.end());

  // Visualize
  if (!inliers_size_t.empty()) {
    PointCloud inliers = cloud.extract(inliers_size_t);
    stream().log(path + "/plane_inliers",
                 adapters::toPoints3DWithColor(inliers, rerun::Color(0, 200, 0))
                     .with_radii({point_radius}));
  }

  if (!outlier_indices.empty()) {
    PointCloud outliers = cloud.extract(outlier_indices);
    stream().log(
        path + "/plane_outliers",
        adapters::toPoints3DWithColor(outliers, rerun::Color(200, 200, 200))
            .with_radii({point_radius}));
  }
}

/// Visualize ground segmentation result
/// @param path Entity path in Rerun
/// @param cloud Source point cloud
/// @param ground_indices Indices of ground points
/// @param point_radius Point display radius
template <typename IndexType>
inline void showGroundSegmentation(const std::string& path,
                                   const PointCloud& cloud,
                                   const std::vector<IndexType>& ground_indices,
                                   float point_radius = 0.02f) {
  // Create ground set for fast lookup
  std::vector<bool> is_ground(cloud.size(), false);
  for (size_t idx : ground_indices) {
    if (idx < cloud.size()) {
      is_ground[idx] = true;
    }
  }

  // Extract ground and non-ground
  std::vector<size_t> non_ground_indices;
  non_ground_indices.reserve(cloud.size() - ground_indices.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (!is_ground[i]) {
      non_ground_indices.push_back(i);
    }
  }

  // Convert to size_t for extract()
  std::vector<size_t> ground_size_t(ground_indices.begin(), ground_indices.end());

  // Visualize
  if (!ground_size_t.empty()) {
    PointCloud ground = cloud.extract(ground_size_t);
    stream().log(
        path + "/ground",
        adapters::toPoints3DWithColor(ground, rerun::Color(139, 90, 43)) // Brown
            .with_radii({point_radius}));
  }

  if (!non_ground_indices.empty()) {
    PointCloud non_ground = cloud.extract(non_ground_indices);
    stream().log(path + "/non_ground",
                 adapters::toPoints3DWithChannels(non_ground)
                     .with_radii({point_radius}));
  }
}

} // namespace nanopcl::rr

#endif // NANOPCL_USE_RERUN

#endif // NANOPCL_VISUALIZATION_RERUN_HELPERS_HPP
