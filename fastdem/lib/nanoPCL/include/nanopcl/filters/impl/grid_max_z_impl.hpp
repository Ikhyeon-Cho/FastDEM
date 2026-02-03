// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_IMPL_GRID_MAX_Z_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_GRID_MAX_Z_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "nanopcl/core/voxel.hpp"

namespace nanopcl {
namespace filters {
namespace detail {

struct IndexedPoint2D {
  uint64_t key;
  uint32_t index;
  float z;
  bool operator<(const IndexedPoint2D& o) const { return key < o.key; }
};

inline std::vector<IndexedPoint2D>& getIndex2DBuffer() {
  thread_local std::vector<IndexedPoint2D> buffer;
  return buffer;
}

// Compute indices of max-Z points per 2D grid cell
inline std::vector<size_t> computeGridMaxZIndices(const PointCloud& cloud, float grid_size) {
  std::vector<size_t> result;
  if (cloud.empty())
    return result;

  const float inv = 1.0f / grid_size;
  auto& idx = getIndex2DBuffer();
  idx.clear();
  idx.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& p = cloud[i];
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
      continue;
    // Use voxel::pack with z=0 for 2D grid key
    idx.push_back({voxel::pack(Point4(p.x(), p.y(), 0.0f, 1.0f), inv), static_cast<uint32_t>(i), p.z()});
  }
  if (idx.empty())
    return result;

  std::sort(idx.begin(), idx.end());

  result.reserve(idx.size() / 4 + 100);

  size_t start = 0;
  while (start < idx.size()) {
    uint64_t key = idx[start].key;
    uint32_t max_idx = idx[start].index;
    float max_z = idx[start].z;

    size_t end = start + 1;
    while (end < idx.size() && idx[end].key == key) {
      if (idx[end].z > max_z) {
        max_z = idx[end].z;
        max_idx = idx[end].index;
      }
      ++end;
    }

    result.push_back(max_idx);
    start = end;
  }

  return result;
}

} // namespace detail

// =============================================================================
// Grid Max-Z (Copy version)
// =============================================================================

inline PointCloud gridMaxZ(const PointCloud& cloud, float grid_size) {
  if (grid_size < voxel::MIN_SIZE || grid_size > voxel::MAX_SIZE) {
    throw std::invalid_argument("grid_size must be in [0.001, 100]");
  }
  return cloud.extract(detail::computeGridMaxZIndices(cloud, grid_size));
}

// =============================================================================
// Grid Max-Z (Move version)
// =============================================================================

inline PointCloud gridMaxZ(PointCloud&& cloud, float grid_size) {
  if (grid_size < voxel::MIN_SIZE || grid_size > voxel::MAX_SIZE) {
    throw std::invalid_argument("grid_size must be in [0.001, 100]");
  }
  return cloud.extract(detail::computeGridMaxZIndices(cloud, grid_size));
}

} // namespace filters
} // namespace nanopcl

#endif // NANOPCL_FILTERS_IMPL_GRID_MAX_Z_IMPL_HPP
