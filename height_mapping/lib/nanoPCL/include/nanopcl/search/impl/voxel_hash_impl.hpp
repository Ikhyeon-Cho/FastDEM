// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation details for VoxelHash.
// Do not include this file directly; include <nanopcl/search/voxel_hash.hpp>

#ifndef NANOPCL_SEARCH_IMPL_VOXEL_HASH_IMPL_HPP
#define NANOPCL_SEARCH_IMPL_VOXEL_HASH_IMPL_HPP

#include <cmath>

namespace npcl {
namespace search {

// =============================================================================
// Build
// =============================================================================

inline void VoxelHash::build(const PointCloud& cloud) {
  build(std::make_shared<PointCloud>(cloud));
}

inline void VoxelHash::build(std::shared_ptr<const PointCloud> cloud) {
  cloud_ = std::move(cloud);
  if (!cloud_) return;

  size_t N = cloud_->size();

  // Prepare hash table (load factor ~0.5 for speed)
  size_t table_size = 1;
  while (table_size < 2 * N) table_size <<= 1;

  hash_table_key_.assign(table_size, VoxelKey::invalid());
  hash_table_head_.assign(table_size, INVALID_INDEX);
  next_node_.resize(N);
  mask_ = table_size - 1;

  // Insert points
  for (uint32_t i = 0; i < N; ++i) {
    const auto& pt = (*cloud_)[i];
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) ||
        !std::isfinite(pt.z())) {
      continue;
    }

    VoxelKey key = VoxelKey::from(pt, inv_resolution_);
    uint64_t hash = key.hash();

    // Linear probing
    size_t slot = hash & mask_;
    while (true) {
      if (!hash_table_key_[slot].isValid()) {
        hash_table_key_[slot] = key;
        hash_table_head_[slot] = i;
        next_node_[i] = INVALID_INDEX;
        break;
      } else if (hash_table_key_[slot] == key) {
        next_node_[i] = hash_table_head_[slot];
        hash_table_head_[slot] = i;
        break;
      }
      slot = (slot + 1) & mask_;
    }
  }
}

// =============================================================================
// Radius Search - Callback (core implementation)
// =============================================================================

template <typename Callback>
void VoxelHash::radius(const Point& center, float r, Callback&& cb) const {
  if (!cloud_) return;

  const float r_sq = r * r;
  const int range = static_cast<int>(std::ceil(r * inv_resolution_));
  VoxelKey center_key = VoxelKey::from(center, inv_resolution_);

  for (int dx = -range; dx <= range; ++dx) {
    for (int dy = -range; dy <= range; ++dy) {
      for (int dz = -range; dz <= range; ++dz) {
        VoxelKey key{center_key.coord + Eigen::Vector3i(dx, dy, dz)};

        uint64_t hash = key.hash();
        size_t slot = hash & mask_;

        while (hash_table_key_[slot].isValid()) {
          if (hash_table_key_[slot] == key) {
            uint32_t idx = hash_table_head_[slot];
            while (idx != INVALID_INDEX) {
              const auto& pt = (*cloud_)[idx];
              float dist_sq = (pt - center).squaredNorm();
              if (dist_sq <= r_sq) {
                cb(idx, pt, dist_sq);
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

// =============================================================================
// Radius Search - Vector out parameter
// =============================================================================

inline void VoxelHash::radius(const Point& center, float r,
                              std::vector<uint32_t>& out) const {
  out.clear();
  radius(center, r,
         [&out](uint32_t idx, const Point&, float) { out.push_back(idx); });
}

// =============================================================================
// Radius Search - Return value (RVO)
// =============================================================================

inline std::vector<uint32_t> VoxelHash::radius(const Point& center,
                                               float r) const {
  std::vector<uint32_t> result;
  radius(center, r, result);
  return result;
}

// =============================================================================
// Nearest Search (Optimized: Layer 0-1 first, then fallback)
// =============================================================================

inline std::optional<NearestResult> VoxelHash::nearest(const Point& center,
                                                       float max_r) const {
  if (!cloud_) return std::nullopt;

  const float max_r_sq = max_r * max_r;
  float best_dist_sq = max_r_sq;
  uint32_t best_idx = INVALID_INDEX;

  const VoxelKey center_key = VoxelKey::from(center, inv_resolution_);

  // Helper: search single voxel
  auto searchVoxel = [&](const VoxelKey& key) {
    uint64_t hash = key.hash();
    size_t slot = hash & mask_;

    while (hash_table_key_[slot].isValid()) {
      if (hash_table_key_[slot] == key) {
        uint32_t idx = hash_table_head_[slot];
        while (idx != INVALID_INDEX) {
          const auto& pt = (*cloud_)[idx];
          float dist_sq = (pt - center).squaredNorm();
          if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_idx = idx;
          }
          idx = next_node_[idx];
        }
        return;
      }
      slot = (slot + 1) & mask_;
    }
  };

  // Layer 0: Center voxel (most likely to contain nearest)
  searchVoxel(center_key);

  // Layer 1: 26 neighbors (hardcoded offsets, no loop overhead)
  static constexpr int L1[26][3] = {
      {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0},
      {-1, 0, 1},   {-1, 1, -1}, {-1, 1, 0},  {-1, 1, 1},  {0, -1, -1},
      {0, -1, 0},   {0, -1, 1},  {0, 0, -1},  {0, 0, 1},   {0, 1, -1},
      {0, 1, 0},    {0, 1, 1},   {1, -1, -1}, {1, -1, 0},  {1, -1, 1},
      {1, 0, -1},   {1, 0, 0},   {1, 0, 1},   {1, 1, -1},  {1, 1, 0},
      {1, 1, 1},
  };

  for (const auto& d : L1) {
    searchVoxel(VoxelKey{center_key.coord + Eigen::Vector3i(d[0], d[1], d[2])});
  }

  // Early exit: if found in Layer 0-1, return immediately
  // (For SLAM/ICP, approximate nearest is sufficient)
  if (best_idx != INVALID_INDEX) {
    return NearestResult{best_idx, best_dist_sq};
  }

  // Fallback: search remaining range with original radius search
  const int range = static_cast<int>(std::ceil(max_r * inv_resolution_));
  if (range <= 1) {
    // Already searched all voxels in range
    return std::nullopt;
  }

  // Search Layer 2+ only
  for (int dx = -range; dx <= range; ++dx) {
    for (int dy = -range; dy <= range; ++dy) {
      for (int dz = -range; dz <= range; ++dz) {
        // Skip Layer 0-1 (already searched)
        if (std::abs(dx) <= 1 && std::abs(dy) <= 1 && std::abs(dz) <= 1) {
          continue;
        }
        searchVoxel(VoxelKey{center_key.coord + Eigen::Vector3i(dx, dy, dz)});
      }
    }
  }

  if (best_idx == INVALID_INDEX) {
    return std::nullopt;
  }

  return NearestResult{best_idx, best_dist_sq};
}

}  // namespace search
}  // namespace npcl

#endif  // NANOPCL_SEARCH_IMPL_VOXEL_HASH_IMPL_HPP
