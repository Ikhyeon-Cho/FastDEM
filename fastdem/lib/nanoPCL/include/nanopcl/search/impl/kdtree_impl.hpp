// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_IMPL_KDTREE_IMPL_HPP
#define NANOPCL_SEARCH_IMPL_KDTREE_IMPL_HPP

namespace nanopcl {
namespace search {

inline void KdTree::build(const PointCloud& cloud) {
  shared_cloud_.reset(); // No ownership

  if (cloud.empty()) {
    cloud_ = nullptr;
    adaptor_.reset();
    index_.reset();
    return;
  }

  cloud_ = &cloud;
  adaptor_ = std::make_unique<PointCloudAdaptor>(cloud_);
  index_ = std::make_unique<KdTreeIndex>(3, *adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
}

inline void KdTree::build(std::shared_ptr<const PointCloud> cloud) {
  shared_cloud_ = std::move(cloud);

  if (!shared_cloud_ || shared_cloud_->empty()) {
    cloud_ = nullptr;
    adaptor_.reset();
    index_.reset();
    return;
  }

  cloud_ = shared_cloud_.get();
  adaptor_ = std::make_unique<PointCloudAdaptor>(cloud_);
  index_ = std::make_unique<KdTreeIndex>(3, *adaptor_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
}

inline void KdTree::radius(const Point& center, float r, std::vector<uint32_t>& out) const {
  out.clear();
  if (!index_) return;

  const float r_sq = r * r;
  nanoflann::SearchParameters params;
  params.sorted = false;

  std::vector<nanoflann::ResultItem<uint32_t, float>> matches;
  index_->radiusSearch(center.data(), r_sq, matches, params);

  out.reserve(matches.size());
  for (const auto& m : matches)
    out.push_back(m.first);
}

inline std::vector<uint32_t> KdTree::radius(const Point& center, float r) const {
  std::vector<uint32_t> result;
  radius(center, r, result);
  return result;
}

inline std::optional<NearestResult> KdTree::nearest(const Point& center, float max_r) const {
  if (!index_) return std::nullopt;

  uint32_t idx;
  float dist_sq;
  nanoflann::KNNResultSet<float, uint32_t> result_set(1);
  result_set.init(&idx, &dist_sq);

  index_->findNeighbors(result_set, center.data());

  if (dist_sq > max_r * max_r) return std::nullopt;
  return NearestResult{idx, dist_sq};
}

inline void KdTree::knn(const Point& center, size_t k, std::vector<NearestResult>& out) const {
  out.clear();
  if (!index_ || k == 0) return;

  k = std::min(k, size());

  std::vector<uint32_t> indices(k);
  std::vector<float> dists_sq(k);

  nanoflann::KNNResultSet<float, uint32_t> result_set(k);
  result_set.init(indices.data(), dists_sq.data());
  index_->findNeighbors(result_set, center.data());

  size_t found = result_set.size();
  out.reserve(found);
  for (size_t i = 0; i < found; ++i) {
    out.push_back(NearestResult{indices[i], dists_sq[i]});
  }
}

inline std::vector<NearestResult> KdTree::knn(const Point& center, size_t k) const {
  std::vector<NearestResult> result;
  knn(center, k, result);
  return result;
}

} // namespace search
} // namespace nanopcl

#endif // NANOPCL_SEARCH_IMPL_KDTREE_IMPL_HPP
