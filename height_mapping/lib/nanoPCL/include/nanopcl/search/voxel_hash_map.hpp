// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_VOXEL_HASH_MAP_HPP
#define NANOPCL_SEARCH_VOXEL_HASH_MAP_HPP

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace search {

namespace detail {
/// Spatial hash primes (Teschner et al., "Optimized Spatial Hashing for Collision Detection")
constexpr uint64_t HASH_PRIME_X = 73856093;
constexpr uint64_t HASH_PRIME_Y = 19349663;
constexpr uint64_t HASH_PRIME_Z = 83492791;
}  // namespace detail

/**
 * @brief High-performance Spatial Hash Map using Linked List in Array.
 *
 * Optimization:
 * 1. No std::unordered_map (Avoids node allocations and pointer chasing).
 * 2. Flat arrays for cache locality.
 * 3. Implicit linked list for handling multiple points per voxel.
 */
class VoxelHashMap {
 public:
  explicit VoxelHashMap(float resolution = 0.5f)
      : resolution_(resolution), inv_resolution_(1.0f / resolution) {}

  /// Build spatial index from shared pointer (zero-copy, recommended)
  void setInputCloud(std::shared_ptr<const PointCloud> cloud);

  /// Build spatial index from reference (creates internal copy for safety)
  void setInputCloud(const PointCloud& cloud);

  /**
   * @brief Find points within radius and execute callback for each
   *
   * Optimization: Avoids std::vector allocation and copying of indices.
   * Useful for direct accumulation (e.g., computing mean/covariance).
   */
  template <typename Callback>
  inline void radiusSearch(const Point& center, float radius,
                           const Callback& callback) const {
    if (!cloud_) return;

    const float r_sq = radius * radius;
    const int range = static_cast<int>(std::ceil(radius * inv_resolution_));
    Eigen::Vector3i center_key = getKey(center);

    for (int dx = -range; dx <= range; ++dx) {
      for (int dy = -range; dy <= range; ++dy) {
        for (int dz = -range; dz <= range; ++dz) {
          Eigen::Vector3i key = center_key + Eigen::Vector3i(dx, dy, dz);

          uint64_t hash = hashKey(key);
          size_t slot = hash & mask_;

          while (hash_table_key_[slot] != INVALID_KEY) {
            if (hash_table_key_[slot] == key) {
              uint32_t idx = hash_table_head_[slot];
              while (idx != INVALID_INDEX) {
                const auto& pt = (*cloud_)[idx];
                if ((pt - center).squaredNorm() <= r_sq) {
                  callback(idx, pt);
                }
                idx = next_node_[idx];
              }
              break;
            }
            slot = (slot + 1) & mask_;
          }
        }
      }
    }
  }

  /// Find points within radius (Index-based)
  size_t radiusSearch(const Point& center, float radius,
                      std::vector<uint32_t>& indices) const;

 private:
  Eigen::Vector3i getKey(const Point& p) const {
    return Eigen::Vector3i(
        static_cast<int>(std::floor(p.x() * inv_resolution_)),
        static_cast<int>(std::floor(p.y() * inv_resolution_)),
        static_cast<int>(std::floor(p.z() * inv_resolution_)));
  }

  // Optimized spatial hash function
  uint64_t hashKey(const Eigen::Vector3i& k) const {
    return ((static_cast<uint64_t>(k[0]) * detail::HASH_PRIME_X) ^
            (static_cast<uint64_t>(k[1]) * detail::HASH_PRIME_Y) ^
            (static_cast<uint64_t>(k[2]) * detail::HASH_PRIME_Z));
  }

  static constexpr uint32_t INVALID_INDEX =
      std::numeric_limits<uint32_t>::max();
  const Eigen::Vector3i INVALID_KEY =
      Eigen::Vector3i::Constant(std::numeric_limits<int>::min());

  float resolution_;
  float inv_resolution_;
  std::shared_ptr<const PointCloud> cloud_;

  // Flat Hash Table
  std::vector<Eigen::Vector3i> hash_table_key_;
  std::vector<uint32_t> hash_table_head_;

  // Linked List (Next pointer for each point)
  std::vector<uint32_t> next_node_;

  size_t mask_ = 0;
};

}  // namespace search
}  // namespace nanopcl

// Include implementation
#include "nanopcl/search/impl/voxel_hash_map_impl.hpp"

#endif  // NANOPCL_SEARCH_VOXEL_HASH_MAP_HPP
