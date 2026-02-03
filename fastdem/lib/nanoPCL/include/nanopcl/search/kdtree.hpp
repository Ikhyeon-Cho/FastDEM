// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_KDTREE_HPP
#define NANOPCL_SEARCH_KDTREE_HPP

#include <memory>
#include <optional>
#include <vector>

#include "nanoflann/nanoflann.hpp"
#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/search/result.hpp"

namespace nanopcl {
namespace search {

/// KD-Tree for KNN/radius search. Best for sparse clouds with single build, many queries.
/// @note The PointCloud must remain valid and unchanged while the tree exists.
class KdTree {
public:
  KdTree() = default;
  ~KdTree() = default;

  KdTree(const KdTree&) = delete;
  KdTree& operator=(const KdTree&) = delete;
  KdTree(KdTree&&) = default;
  KdTree& operator=(KdTree&&) = default;

  /// Build from reference (no copy, caller must keep cloud alive)
  void build(const PointCloud& cloud);

  /// Build with shared ownership (extends cloud lifetime)
  void build(std::shared_ptr<const PointCloud> cloud);

  // Radius Search
  [[nodiscard]] std::vector<uint32_t> radius(const Point& center, float r) const;
  void radius(const Point& center, float r, std::vector<uint32_t>& out) const;

  // Nearest Search
  [[nodiscard]] std::optional<NearestResult> nearest(const Point& center, float max_r) const;

  // KNN Search
  [[nodiscard]] std::vector<NearestResult> knn(const Point& center, size_t k) const;
  void knn(const Point& center, size_t k, std::vector<NearestResult>& out) const;

  // Accessors
  [[nodiscard]] bool empty() const noexcept { return !cloud_ || cloud_->empty(); }
  [[nodiscard]] size_t size() const noexcept { return cloud_ ? cloud_->size() : 0; }

private:
  struct PointCloudAdaptor {
    const PointCloud* cloud;
    PointCloudAdaptor(const PointCloud* c)
        : cloud(c) {}
    size_t kdtree_get_point_count() const { return cloud ? cloud->size() : 0; }
    float kdtree_get_pt(size_t idx, size_t dim) const { return (*cloud)[idx][dim]; }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
  };

  using KdTreeIndex = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
      PointCloudAdaptor,
      3,
      uint32_t>;

  const PointCloud* cloud_ = nullptr;              // Non-owning (fast)
  std::shared_ptr<const PointCloud> shared_cloud_; // Optional ownership extension
  std::unique_ptr<PointCloudAdaptor> adaptor_;
  std::unique_ptr<KdTreeIndex> index_;
};

} // namespace search
} // namespace nanopcl

#include "nanopcl/search/impl/kdtree_impl.hpp"

#endif // NANOPCL_SEARCH_KDTREE_HPP
