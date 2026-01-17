// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation details for KdTree.
// Do not include this file directly; include <nanopcl/search/kdtree.hpp>

#ifndef NANOPCL_SEARCH_IMPL_KDTREE_IMPL_HPP
#define NANOPCL_SEARCH_IMPL_KDTREE_IMPL_HPP

namespace npcl {
namespace search {

// =============================================================================
// Build
// =============================================================================

inline void KdTree::build(const PointCloud& cloud) {
  build(std::make_shared<PointCloud>(cloud));
}

inline void KdTree::build(std::shared_ptr<const PointCloud> cloud) {
  cloud_ = std::move(cloud);

  if (!cloud_ || cloud_->empty()) {
    adaptor_.reset();
    index_.reset();
    return;
  }

  // Create adaptor
  adaptor_ = std::make_unique<PointCloudAdaptor>(cloud_.get());

  // Create index with default leaf size
  index_ = std::make_unique<KdTreeIndex>(
      3,  // dimensions
      *adaptor_,
      nanoflann::KDTreeSingleIndexAdaptorParams(10)  // max leaf size
  );
}

// =============================================================================
// Radius Search
// =============================================================================

inline void KdTree::radius(const Point& center, float r,
                           std::vector<uint32_t>& out) const {
  out.clear();
  if (!index_) return;

  const float r_sq = r * r;
  nanoflann::SearchParameters params;
  params.sorted = false;

  std::vector<nanoflann::ResultItem<uint32_t, float>> matches;
  index_->radiusSearch(center.data(), r_sq, matches, params);

  out.reserve(matches.size());
  for (const auto& match : matches) {
    out.push_back(match.first);
  }
}

inline std::vector<uint32_t> KdTree::radius(const Point& center,
                                            float r) const {
  std::vector<uint32_t> result;
  radius(center, r, result);
  return result;
}

// =============================================================================
// Nearest Search
// =============================================================================

inline std::optional<NearestResult> KdTree::nearest(const Point& center,
                                                    float max_r) const {
  if (!index_) return std::nullopt;

  uint32_t idx;
  float dist_sq;
  nanoflann::KNNResultSet<float, uint32_t> result_set(1);
  result_set.init(&idx, &dist_sq);

  index_->findNeighbors(result_set, center.data());

  // Check if within max_r
  const float max_r_sq = max_r * max_r;
  if (dist_sq > max_r_sq) {
    return std::nullopt;
  }

  return NearestResult{idx, dist_sq};
}

// =============================================================================
// KNN Search
// =============================================================================

inline void KdTree::knn(const Point& center, size_t k,
                        std::vector<NearestResult>& out) const {
  out.clear();
  if (!index_ || k == 0) return;

  // Clamp k to cloud size
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

inline std::vector<NearestResult> KdTree::knn(const Point& center,
                                              size_t k) const {
  std::vector<NearestResult> result;
  knn(center, k, result);
  return result;
}

}  // namespace search
}  // namespace npcl

#endif  // NANOPCL_SEARCH_IMPL_KDTREE_IMPL_HPP
