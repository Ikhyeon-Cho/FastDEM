// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_KDTREE_HPP
#define NANOPCL_SEARCH_KDTREE_HPP

#include <memory>
#include <optional>
#include <vector>

#include "nanoflann/nanoflann.hpp"
#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/search/result.hpp"

namespace npcl {
namespace search {

/**
 * @brief KD-Tree for efficient nearest neighbor search
 *
 * Wrapper around nanoflann providing:
 * - K-nearest neighbor (KNN) search
 * - Radius search
 * - Nearest single point search
 *
 * Best suited for:
 * - Single build, many queries
 * - KNN queries (ICP, feature matching)
 * - Sparse point clouds
 */
class KdTree {
 public:
  KdTree() = default;
  ~KdTree() = default;

  // Non-copyable (owns nanoflann index)
  KdTree(const KdTree&) = delete;
  KdTree& operator=(const KdTree&) = delete;
  KdTree(KdTree&&) = default;
  KdTree& operator=(KdTree&&) = default;

  // ===========================================================================
  // Build
  // ===========================================================================

  /**
   * @brief Build KD-tree index from point cloud
   *
   * @warning The tree stores a **reference** to the cloud, not a copy.
   *          The `cloud` object must remain valid and **unchanged** while the tree exists.
   *          Modifying the cloud (adding/removing points) after building invalidates the tree.
   *
   * @param cloud Reference to the point cloud
   */
  void build(const PointCloud& cloud);

  /// Build KD-tree index from shared pointer (zero-copy, keeps cloud alive)
  void build(std::shared_ptr<const PointCloud> cloud);

  // ===========================================================================
  // Radius Search
  // ===========================================================================

  /// Find points within radius - convenience API (RVO)
  [[nodiscard]] std::vector<uint32_t> radius(const Point& center,
                                             float r) const;

  /// Find points within radius - performance API (reuse vector)
  void radius(const Point& center, float r, std::vector<uint32_t>& out) const;

  // ===========================================================================
  // Nearest Search
  // ===========================================================================

  /// Find nearest point within max_r
  [[nodiscard]] std::optional<NearestResult> nearest(const Point& center,
                                                     float max_r) const;

  // ===========================================================================
  // KNN Search (KdTree specific)
  // ===========================================================================

  /**
   * @brief Find k nearest neighbors
   *
   * @param center Query point
   * @param k Number of neighbors to find
   * @return Vector of results sorted by distance (nearest first)
   *
   * @code
   *   auto neighbors = tree.knn(query_pt, 5);
   *   for (const auto& nb : neighbors) {
   *     // Access neighbor: cloud[nb.index]
   *     // Distance squared: nb.dist_sq
   *   }
   * @endcode
   */
  [[nodiscard]] std::vector<NearestResult> knn(const Point& center,
                                               size_t k) const;

  /// Find k nearest neighbors - performance API (reuse vector)
  void knn(const Point& center, size_t k,
           std::vector<NearestResult>& out) const;

  // ===========================================================================
  // Accessors
  // ===========================================================================

  [[nodiscard]] bool empty() const noexcept {
    return !cloud_ || cloud_->empty();
  }
  [[nodiscard]] size_t size() const noexcept {
    return cloud_ ? cloud_->size() : 0;
  }

 private:
  /// Adaptor for nanoflann to access nanoPCL::PointCloud
  struct PointCloudAdaptor {
    const PointCloud* cloud;

    PointCloudAdaptor(const PointCloud* c) : cloud(c) {}

    /// Number of points
    inline size_t kdtree_get_point_count() const {
      return cloud ? cloud->size() : 0;
    }

    /// Get coordinate value at dimension dim for point idx
    inline float kdtree_get_pt(size_t idx, size_t dim) const {
      return (*cloud)[idx][dim];
    }

    /// Optional bounding box (returning false uses default)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
      return false;
    }
  };

  using KdTreeIndex = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>, PointCloudAdaptor,
      3,        // dimensions
      uint32_t  // index type
      >;

  std::shared_ptr<const PointCloud> cloud_;
  std::unique_ptr<PointCloudAdaptor> adaptor_;
  std::unique_ptr<KdTreeIndex> index_;
};

}  // namespace search
}  // namespace npcl

#include "nanopcl/search/impl/kdtree_impl.hpp"

#endif  // NANOPCL_SEARCH_KDTREE_HPP
