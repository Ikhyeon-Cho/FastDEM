// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_VOXEL_HASH_HPP
#define NANOPCL_SEARCH_VOXEL_HASH_HPP

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/core/voxel_key.hpp"
#include "nanopcl/search/result.hpp"

namespace npcl {
namespace search {

/**
 * @brief High-performance spatial hash map for radius search
 *
 * Optimized for:
 * - Fast build time O(N)
 * - Efficient radius queries O(1) average
 * - Cache-friendly flat array storage
 *
 * Best suited for:
 * - Real-time applications with frequent rebuilds
 * - Radius search queries
 * - Dense point clouds
 */
class VoxelHash {
 public:
  explicit VoxelHash(float resolution = 0.5f)
      : resolution_(resolution), inv_resolution_(1.0f / resolution) {}

  // ===========================================================================
  // Build
  // ===========================================================================

  /// Build spatial index from reference (creates internal copy)
  void build(const PointCloud& cloud);

  /// Build spatial index from shared pointer (zero-copy)
  void build(std::shared_ptr<const PointCloud> cloud);

  // ===========================================================================
  // Radius Search
  // ===========================================================================

  /// Find points within radius - convenience API (RVO)
  [[nodiscard]] std::vector<uint32_t> radius(const Point& center,
                                             float r) const;

  /// Find points within radius - performance API (reuse vector)
  void radius(const Point& center, float r, std::vector<uint32_t>& out) const;

  /**
   * @brief Find points within radius with callback (zero-allocation)
   *
   * @param center Query point
   * @param r Search radius
   * @param cb Callback function: `void(uint32_t idx, const Point& p, float dist_sq)`
   *
   * @note **Performance Warning**: This method iterates over all voxels intersecting
   *       the search sphere. If `r` is much larger than `resolution`, it becomes
   *       inefficient (O(r^3)). Best used for small radius queries relative to resolution.
   */
  template <typename Callback>
  void radius(const Point& center, float r, Callback&& cb) const;

  // ===========================================================================
  // Nearest Search
  // ===========================================================================

  /**
   * @brief Find nearest point within max_r
   *
   * Uses a **Layered Search** strategy for efficiency:
   * 1. Check the voxel containing the query point (Layer 0).
   * 2. If not found, check the immediate 26 neighbors (Layer 1).
   * 3. If still not found, expand outwards layer by layer.
   *
   * This is optimized for scenarios where the nearest neighbor is expected to be close
   * (e.g., ICP registration).
   *
   * @param center Query point
   * @param max_r Maximum search radius (required to bound the search space)
   * @return NearestResult if found, nullopt otherwise
   */
  [[nodiscard]] std::optional<NearestResult> nearest(const PointCloud& center,
                                                     float max_r) const;
  // Note: The original code had `const Point& center`. I will correct the parameter type in the replacement.
  [[nodiscard]] std::optional<NearestResult> nearest(const Point& center,
                                                     float max_r) const;

  // ===========================================================================
  // Accessors
  // ===========================================================================

  [[nodiscard]] float resolution() const noexcept { return resolution_; }
  [[nodiscard]] float invResolution() const noexcept { return inv_resolution_; }
  [[nodiscard]] bool empty() const noexcept {
    return !cloud_ || cloud_->empty();
  }
  [[nodiscard]] size_t size() const noexcept {
    return cloud_ ? cloud_->size() : 0;
  }

 private:
  static constexpr uint32_t INVALID_INDEX =
      std::numeric_limits<uint32_t>::max();

  float resolution_;
  float inv_resolution_;
  std::shared_ptr<const PointCloud> cloud_;

  // Flat Hash Table
  std::vector<VoxelKey> hash_table_key_;
  std::vector<uint32_t> hash_table_head_;

  // Linked List (next pointer for each point)
  std::vector<uint32_t> next_node_;

  size_t mask_ = 0;
};

}  // namespace search
}  // namespace npcl

#include "nanopcl/search/impl/voxel_hash_impl.hpp"

#endif  // NANOPCL_SEARCH_VOXEL_HASH_HPP
