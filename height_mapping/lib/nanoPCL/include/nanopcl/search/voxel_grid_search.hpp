// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_VOXEL_GRID_SEARCH_HPP
#define NANOPCL_SEARCH_VOXEL_GRID_SEARCH_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace search {

/**
 * @brief High-performance spatial search using sorted voxel grid.
 *
 * Uses a flat, sorted vector instead of hash map to maximize cache locality
 * and avoid memory allocation overhead during search.
 *
 * - Build: O(N log N) (Sorting)
 * - Search: O(log N) per voxel check (Binary Search)
 *
 * Optimized for dense point clouds and radius search.
 */
class VoxelGridSearch {
 public:
  explicit VoxelGridSearch(float resolution = 0.5f)
      : resolution_(resolution), inv_resolution_(1.0f / resolution) {}

  void setInputCloud(const PointCloud& cloud) {
    cloud_ptr_ = &cloud;
    indexed_points_.resize(cloud.size());

    // 1. Compute keys
    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto& pt = cloud[i];
      if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z())) {
        indexed_points_[i] = {std::numeric_limits<uint64_t>::max(), static_cast<uint32_t>(i)};
        continue;
      }
      indexed_points_[i] = {computeKey(pt), static_cast<uint32_t>(i)};
    }

    // 2. Sort by key (The secret sauce for performance)
    std::sort(indexed_points_.begin(), indexed_points_.end());
  }

  /**
   * @brief Find points within radius
   */
  size_t radiusSearch(const Point& center, float radius,
                      std::vector<uint32_t>& indices) const {
    indices.clear();
    if (!cloud_ptr_) return 0;

    const float r_sq = radius * radius;
    const int range = static_cast<int>(std::ceil(radius * inv_resolution_));
    
    // Base voxel coordinates
    int32_t cx = static_cast<int32_t>(std::floor(center.x() * inv_resolution_));
    int32_t cy = static_cast<int32_t>(std::floor(center.y() * inv_resolution_));
    int32_t cz = static_cast<int32_t>(std::floor(center.z() * inv_resolution_));

    // Check neighbor voxels
    for (int dx = -range; dx <= range; ++dx) {
      for (int dy = -range; dy <= range; ++dy) {
        for (int dz = -range; dz <= range; ++dz) {
          uint64_t key = computeKey(cx + dx, cy + dy, cz + dz);
          
          // Binary search for the voxel block
          // We use equal_range to find the start and end of points in this voxel
          auto range_it = std::equal_range(
              indexed_points_.begin(), indexed_points_.end(), IndexedPoint{key, 0},
              [](const IndexedPoint& a, const IndexedPoint& b) {
                return a.key < b.key;
              });

          // Iterate through points in this voxel
          for (auto it = range_it.first; it != range_it.second; ++it) {
            const auto& pt = (*cloud_ptr_)[it->index];
            if ((pt - center).squaredNorm() <= r_sq) {
              indices.push_back(it->index);
            }
          }
        }
      }
    }
    return indices.size();
  }

 private:
  struct IndexedPoint {
    uint64_t key;
    uint32_t index;

    bool operator<(const IndexedPoint& other) const { return key < other.key; }
  };

  inline uint64_t computeKey(const Point& p) const {
    int32_t ix = static_cast<int32_t>(std::floor(p.x() * inv_resolution_));
    int32_t iy = static_cast<int32_t>(std::floor(p.y() * inv_resolution_));
    int32_t iz = static_cast<int32_t>(std::floor(p.z() * inv_resolution_));
    return computeKey(ix, iy, iz);
  }

  inline uint64_t computeKey(int32_t ix, int32_t iy, int32_t iz) const {
    constexpr int32_t offset = 1 << 20;  // 1M offset
    uint64_t ux = static_cast<uint64_t>(ix + offset);
    uint64_t uy = static_cast<uint64_t>(iy + offset);
    uint64_t uz = static_cast<uint64_t>(iz + offset);
    return (ux << 42) | (uy << 21) | uz;
  }

  float resolution_;
  float inv_resolution_;
  const PointCloud* cloud_ptr_ = nullptr;
  std::vector<IndexedPoint> indexed_points_;
};

}  // namespace search
}  // namespace nanopcl

#endif  // NANOPCL_SEARCH_VOXEL_GRID_SEARCH_HPP
