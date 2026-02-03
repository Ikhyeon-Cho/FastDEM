// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_IMPL_VOXEL_HASH_IMPL_HPP
#define NANOPCL_SEARCH_IMPL_VOXEL_HASH_IMPL_HPP

#include <cmath>
#include <iostream>

namespace nanopcl {
namespace search {

inline void VoxelHash::build(const PointCloud& cloud) {
  shared_cloud_.reset(); // No ownership

  if (cloud.empty()) {
    cloud_ = nullptr;
    hash_table_key_.clear();
    hash_table_head_.clear();
    next_node_.clear();
    mask_ = 0;
    shift_bits_ = 0;
    return;
  }

  cloud_ = &cloud;

  size_t N = cloud_->size();

  // Hash table with ~50% load factor
  size_t table_size = 1;
  while (table_size < 2 * N)
    table_size <<= 1;

  hash_table_key_.assign(table_size, voxel::INVALID_KEY);
  hash_table_head_.assign(table_size, INVALID_INDEX);
  next_node_.resize(N);
  mask_ = table_size - 1;
  shift_bits_ = 64 - static_cast<size_t>(__builtin_ctzll(table_size));

  // Insert points (linear probing with Fibonacci hashing)
  for (uint32_t i = 0; i < N; ++i) {
    const auto& pt = (*cloud_)[i];
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z()))
      continue;

    uint64_t key = voxel::pack(pt, inv_resolution_);
    size_t slot = hashSlot(key);

    while (true) {
      if (!voxel::isValid(hash_table_key_[slot])) {
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

inline void VoxelHash::build(std::shared_ptr<const PointCloud> cloud) {
  shared_cloud_ = std::move(cloud);

  if (!shared_cloud_ || shared_cloud_->empty()) {
    cloud_ = nullptr;
    hash_table_key_.clear();
    hash_table_head_.clear();
    next_node_.clear();
    mask_ = 0;
    shift_bits_ = 0;
    return;
  }

  cloud_ = shared_cloud_.get();
  size_t N = cloud_->size();

  // Hash table with ~50% load factor
  size_t table_size = 1;
  while (table_size < 2 * N)
    table_size <<= 1;

  hash_table_key_.assign(table_size, voxel::INVALID_KEY);
  hash_table_head_.assign(table_size, INVALID_INDEX);
  next_node_.resize(N);
  mask_ = table_size - 1;
  shift_bits_ = 64 - static_cast<size_t>(__builtin_ctzll(table_size));

  // Insert points (linear probing with Fibonacci hashing)
  for (uint32_t i = 0; i < N; ++i) {
    const auto& pt = (*cloud_)[i];
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z()))
      continue;

    uint64_t key = voxel::pack(pt, inv_resolution_);
    size_t slot = hashSlot(key);

    while (true) {
      if (!voxel::isValid(hash_table_key_[slot])) {
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

template <typename Callback>
void VoxelHash::radius(const Point& center, float r, Callback&& cb) const {
  if (!cloud_) return;

  const float r_sq = r * r;
  const int range = static_cast<int>(std::ceil(r * inv_resolution_));

  // Warn if search range is too large (O(range^3) complexity)
  if (range > 15) {
    static bool warned = false;
    if (!warned) {
      std::cerr << "[nanoPCL] Warning: VoxelHash radius search range is large ("
                << range << " voxels). Consider using KdTree for better performance.\n";
      warned = true;
    }
  }

  uint64_t center_key = voxel::pack(center, inv_resolution_);

  for (int dx = -range; dx <= range; ++dx) {
    for (int dy = -range; dy <= range; ++dy) {
      for (int dz = -range; dz <= range; ++dz) {
        uint64_t key = voxel::neighbor(center_key, dx, dy, dz);
        size_t slot = hashSlot(key);

        while (voxel::isValid(hash_table_key_[slot])) {
          if (hash_table_key_[slot] == key) {
            uint32_t idx = hash_table_head_[slot];
            while (idx != INVALID_INDEX) {
              const auto& pt = (*cloud_)[idx];
              float dist_sq = (pt.head<3>() - center).squaredNorm();
              if (dist_sq <= r_sq) {
                cb(idx, pt.head<3>(), dist_sq);
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

inline void VoxelHash::radius(const Point& center, float r, std::vector<uint32_t>& out) const {
  out.clear();
  radius(center, r, [&out](uint32_t idx, const Point&, float) { out.push_back(idx); });
}

inline std::vector<uint32_t> VoxelHash::radius(const Point& center, float r) const {
  std::vector<uint32_t> result;
  radius(center, r, result);
  return result;
}

inline std::optional<NearestResult> VoxelHash::nearest(const Point& center, float max_r) const {
  if (!cloud_) return std::nullopt;

  const float max_r_sq = max_r * max_r;
  float best_dist_sq = max_r_sq;
  uint32_t best_idx = INVALID_INDEX;
  const uint64_t center_key = voxel::pack(center, inv_resolution_);

  auto searchVoxel = [&](uint64_t key) {
    size_t slot = hashSlot(key);
    while (voxel::isValid(hash_table_key_[slot])) {
      if (hash_table_key_[slot] == key) {
        uint32_t idx = hash_table_head_[slot];
        while (idx != INVALID_INDEX) {
          const auto& pt = (*cloud_)[idx];
          float dist_sq = (pt.head<3>() - center).squaredNorm();
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

  // Layer 0: center voxel
  searchVoxel(center_key);

  // Layer 1: 26 neighbors
  static constexpr int L1[26][3] = {
      {-1, -1, -1},
      {-1, -1, 0},
      {-1, -1, 1},
      {-1, 0, -1},
      {-1, 0, 0},
      {-1, 0, 1},
      {-1, 1, -1},
      {-1, 1, 0},
      {-1, 1, 1},
      {0, -1, -1},
      {0, -1, 0},
      {0, -1, 1},
      {0, 0, -1},
      {0, 0, 1},
      {0, 1, -1},
      {0, 1, 0},
      {0, 1, 1},
      {1, -1, -1},
      {1, -1, 0},
      {1, -1, 1},
      {1, 0, -1},
      {1, 0, 0},
      {1, 0, 1},
      {1, 1, -1},
      {1, 1, 0},
      {1, 1, 1},
  };
  for (const auto& d : L1) {
    searchVoxel(voxel::neighbor(center_key, d[0], d[1], d[2]));
  }

  // Early exit for approximate nearest (sufficient for ICP/SLAM)
  if (best_idx != INVALID_INDEX) {
    return NearestResult{best_idx, best_dist_sq};
  }

  // Layer 2+: expand if needed
  const int range = static_cast<int>(std::ceil(max_r * inv_resolution_));
  if (range <= 1) return std::nullopt;

  for (int dx = -range; dx <= range; ++dx) {
    for (int dy = -range; dy <= range; ++dy) {
      for (int dz = -range; dz <= range; ++dz) {
        if (std::abs(dx) <= 1 && std::abs(dy) <= 1 && std::abs(dz) <= 1) continue;
        searchVoxel(voxel::neighbor(center_key, dx, dy, dz));
      }
    }
  }

  if (best_idx == INVALID_INDEX) return std::nullopt;
  return NearestResult{best_idx, best_dist_sq};
}

} // namespace search
} // namespace nanopcl

#endif // NANOPCL_SEARCH_IMPL_VOXEL_HASH_IMPL_HPP
