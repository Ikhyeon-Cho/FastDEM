// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_VOXEL_HASH_HPP
#define NANOPCL_SEARCH_VOXEL_HASH_HPP

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/core/voxel.hpp"
#include "nanopcl/search/result.hpp"

namespace nanopcl {
namespace search {

/// Spatial hash map for O(1) radius search. Best for dense clouds with frequent rebuilds.
/// @note The PointCloud must remain valid and unchanged while the hash exists.
class VoxelHash {
public:
  explicit VoxelHash(float resolution = 0.5f)
      : resolution_(resolution), inv_resolution_(1.0f / resolution) {}

  /// Build from reference (no copy, caller must keep cloud alive)
  void build(const PointCloud& cloud);

  /// Build with shared ownership (extends cloud lifetime)
  void build(std::shared_ptr<const PointCloud> cloud);

  // Radius Search
  [[nodiscard]] std::vector<uint32_t> radius(const Point& center, float r) const;
  void radius(const Point& center, float r, std::vector<uint32_t>& out) const;

  /// Callback version: void(uint32_t idx, const Point& p, float dist_sq)
  /// Warning: O(r^3) when r >> resolution
  template <typename Callback>
  void radius(const Point& center, float r, Callback&& cb) const;

  // Nearest Search (Layered: center voxel -> 26 neighbors -> expand)
  [[nodiscard]] std::optional<NearestResult> nearest(const Point& center, float max_r) const;

  // Accessors
  [[nodiscard]] float resolution() const noexcept { return resolution_; }
  [[nodiscard]] bool empty() const noexcept { return !cloud_ || cloud_->empty(); }
  [[nodiscard]] size_t size() const noexcept { return cloud_ ? cloud_->size() : 0; }

private:
  static constexpr uint32_t INVALID_INDEX = std::numeric_limits<uint32_t>::max();

  float resolution_;
  float inv_resolution_;
  const PointCloud* cloud_ = nullptr;              // Non-owning (fast)
  std::shared_ptr<const PointCloud> shared_cloud_; // Optional ownership extension

  /// Fibonacci hashing for uniform distribution
  static constexpr uint64_t HASH_MULTIPLIER = 0x9e3779b97f4a7c15ULL;
  inline size_t hashSlot(uint64_t key) const {
    return (key * HASH_MULTIPLIER) >> shift_bits_;
  }

  // Flat hash table (linear probing)
  std::vector<uint64_t> hash_table_key_;
  std::vector<uint32_t> hash_table_head_;
  std::vector<uint32_t> next_node_; // Linked list per voxel
  size_t mask_ = 0;
  size_t shift_bits_ = 0; // 64 - log2(table_size) for Fibonacci hashing
};

} // namespace search
} // namespace nanopcl

#include "nanopcl/search/impl/voxel_hash_impl.hpp"

#endif // NANOPCL_SEARCH_VOXEL_HASH_HPP
