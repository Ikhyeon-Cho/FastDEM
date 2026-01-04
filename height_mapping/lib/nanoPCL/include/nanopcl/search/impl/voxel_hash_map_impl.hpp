// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for VoxelHashMap.
// Do not include this file directly; include <nanopcl/search/voxel_hash_map.hpp>

#ifndef NANOPCL_SEARCH_IMPL_VOXEL_HASH_MAP_IMPL_HPP
#define NANOPCL_SEARCH_IMPL_VOXEL_HASH_MAP_IMPL_HPP

#include <cmath>

namespace nanopcl {
namespace search {

inline void VoxelHashMap::setInputCloud(std::shared_ptr<const PointCloud> cloud) {
  cloud_ = std::move(cloud);
  if (!cloud_) return;

  size_t N = cloud_->size();

  // 1. Prepare hash table (Load factor ~0.5 for speed)
  // Using a power of 2 size allows bitwise masking instead of modulo
  size_t table_size = 1;
  while (table_size < 2 * N) table_size <<= 1;

  hash_table_key_.assign(table_size, INVALID_KEY);
  hash_table_head_.assign(table_size, INVALID_INDEX);

  // 2. Prepare linked list for collisions/multi-points
  next_node_.resize(N);

  // 3. Insert points
  mask_ = table_size - 1;

  for (uint32_t i = 0; i < N; ++i) {
    const auto& pt = (*cloud_)[i];
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z())) {
      continue;
    }

    Eigen::Vector3i key = getKey(pt);
    uint64_t hash = hashKey(key);

    // Linear Probing to find the bucket for this voxel key
    size_t slot = hash & mask_;
    while (true) {
      if (hash_table_key_[slot] == INVALID_KEY) {
        // New voxel found
        hash_table_key_[slot] = key;
        hash_table_head_[slot] = i;
        next_node_[i] = INVALID_INDEX; // End of list
        break;
      } else if (hash_table_key_[slot] == key) {
        // Existing voxel found, prepend to list
        next_node_[i] = hash_table_head_[slot];
        hash_table_head_[slot] = i;
        break;
      }
      // Collision: Try next slot
      slot = (slot + 1) & mask_;
    }
  }
}

inline void VoxelHashMap::setInputCloud(const PointCloud& cloud) {
  setInputCloud(std::make_shared<PointCloud>(cloud));
}

inline size_t VoxelHashMap::radiusSearch(const Point& center, float radius,
                                          std::vector<uint32_t>& indices) const {
  indices.clear();
  if (!cloud_) return 0;

  const float r_sq = radius * radius;
  const int range = static_cast<int>(std::ceil(radius * inv_resolution_));
  Eigen::Vector3i center_key = getKey(center);

  for (int dx = -range; dx <= range; ++dx) {
    for (int dy = -range; dy <= range; ++dy) {
      for (int dz = -range; dz <= range; ++dz) {
        Eigen::Vector3i key = center_key + Eigen::Vector3i(dx, dy, dz);

        // Find the bucket
        uint64_t hash = hashKey(key);
        size_t slot = hash & mask_;

        while (hash_table_key_[slot] != INVALID_KEY) {
          if (hash_table_key_[slot] == key) {
            // Found voxel, iterate through linked list
            uint32_t idx = hash_table_head_[slot];
            while (idx != INVALID_INDEX) {
              const auto& pt = (*cloud_)[idx];
              if ((pt - center).squaredNorm() <= r_sq) {
                indices.push_back(idx);
              }
              idx = next_node_[idx];
            }
            break; // Done with this voxel
          }
          slot = (slot + 1) & mask_;
        }
      }
    }
  }
  return indices.size();
}

}  // namespace search
}  // namespace nanopcl

#endif  // NANOPCL_SEARCH_IMPL_VOXEL_HASH_MAP_IMPL_HPP
