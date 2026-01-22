// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for 2D grid max-Z filter.
// Do not include this file directly; include <nanopcl/filters/downsample.hpp>

#ifndef NANOPCL_FILTERS_IMPL_GRID_MAX_Z_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_GRID_MAX_Z_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace npcl {
namespace filters {
namespace detail {

// 2D grid coordinate limits (32 bits per axis)
// Valid range after offset: [0, 2^31 - 1]
// With offset of 2^30, original range: [-1073741824, 1073741823]
// At grid_size=0.1m, this covers ~±107,374 km
constexpr int32_t GRID2D_COORD_OFFSET = 1 << 30;
constexpr int32_t GRID2D_COORD_MIN = -GRID2D_COORD_OFFSET;
constexpr int32_t GRID2D_COORD_MAX = GRID2D_COORD_OFFSET - 1;

// Indexed point for 2D grid (stores key, original index, and z for comparison)
struct IndexedPoint2D {
  uint64_t key;
  uint32_t index;
  float z;

  bool operator<(const IndexedPoint2D& other) const { return key < other.key; }
};

inline uint64_t computeGrid2DKey(float x, float y, float inv_grid_size) {
  int32_t ix = static_cast<int32_t>(std::floor(x * inv_grid_size));
  int32_t iy = static_cast<int32_t>(std::floor(y * inv_grid_size));

  // Clamp to valid range
  ix = std::clamp(ix, GRID2D_COORD_MIN, GRID2D_COORD_MAX);
  iy = std::clamp(iy, GRID2D_COORD_MIN, GRID2D_COORD_MAX);

  // Offset to handle negative indices and pack into 64-bit key
  uint64_t ux = static_cast<uint64_t>(static_cast<uint32_t>(ix + GRID2D_COORD_OFFSET));
  uint64_t uy = static_cast<uint64_t>(static_cast<uint32_t>(iy + GRID2D_COORD_OFFSET));

  return (ux << 32) | uy;
}

// Thread-local buffer for indexed points (avoids repeated allocation)
inline std::vector<IndexedPoint2D>& getIndexedPoints2DBuffer() {
  thread_local std::vector<IndexedPoint2D> buffer;
  return buffer;
}

}  // namespace detail

inline PointCloud gridMaxZ(const PointCloud& cloud, float grid_size) {
  if (grid_size < detail::MIN_VOXEL_SIZE || grid_size > 100.0f) {
    throw std::invalid_argument(
        "grid_size must be in [0.001, 100]. "
        "Spatial range is limited by coordinate packing.");
  }

  PointCloud output;
  output.setTimestamp(cloud.timestamp());
  output.setFrameId(cloud.frameId());

  if (cloud.empty()) {
    return output;
  }

  // Cache channel presence
  const bool has_intensity = cloud.hasIntensity();
  const bool has_time = cloud.hasTime();
  const bool has_ring = cloud.hasRing();
  const bool has_color = cloud.hasColor();
  const bool has_label = cloud.hasLabel();

  // Step 1: Compute 2D grid keys
  const float inv_grid_size = 1.0f / grid_size;
  auto& indexed_points = detail::getIndexedPoints2DBuffer();
  indexed_points.clear();
  indexed_points.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }
    uint64_t key = detail::computeGrid2DKey(point.x(), point.y(), inv_grid_size);
    indexed_points.push_back({key, static_cast<uint32_t>(i), point.z()});
  }

  if (indexed_points.empty()) {
    return output;
  }

  // Step 2: Sort by 2D grid key
  std::sort(indexed_points.begin(), indexed_points.end());

  // Step 3: Prepare output capacity
  size_t estimated_size = indexed_points.size() / 4;
  if (estimated_size < 100) estimated_size = 100;
  output.reserve(estimated_size);

  if (has_intensity) output.enableIntensity();
  if (has_time) output.enableTime();
  if (has_ring) output.enableRing();
  if (has_color) output.enableColor();
  if (has_label) output.enableLabel();

  // Step 4: For each cell, find point with max Z
  size_t group_start = 0;
  while (group_start < indexed_points.size()) {
    uint64_t current_key = indexed_points[group_start].key;

    // Find point with max Z in this cell
    uint32_t max_z_idx = indexed_points[group_start].index;
    float max_z = indexed_points[group_start].z;

    size_t group_end = group_start + 1;
    while (group_end < indexed_points.size() &&
           indexed_points[group_end].key == current_key) {
      if (indexed_points[group_end].z > max_z) {
        max_z = indexed_points[group_end].z;
        max_z_idx = indexed_points[group_end].index;
      }
      ++group_end;
    }

    // Add the point with max Z
    output.add(cloud[max_z_idx]);
    detail::copyAttributes(cloud, max_z_idx, output, has_intensity, has_time,
                           has_ring, has_color, has_label);

    group_start = group_end;
  }

  return output;
}

inline PointCloud gridMaxZ(PointCloud&& cloud, float grid_size) {
  if (grid_size < detail::MIN_VOXEL_SIZE || grid_size > 100.0f) {
    throw std::invalid_argument(
        "grid_size must be in [0.001, 100]. "
        "Spatial range is limited by coordinate packing.");
  }

  if (cloud.empty()) {
    return std::move(cloud);
  }

  // Cache channel presence
  const bool has_intensity = cloud.hasIntensity();
  const bool has_time = cloud.hasTime();
  const bool has_ring = cloud.hasRing();
  const bool has_color = cloud.hasColor();
  const bool has_label = cloud.hasLabel();

  // Step 1: Compute 2D grid keys
  const float inv_grid_size = 1.0f / grid_size;
  auto& indexed_points = detail::getIndexedPoints2DBuffer();
  indexed_points.clear();
  indexed_points.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }
    uint64_t key = detail::computeGrid2DKey(point.x(), point.y(), inv_grid_size);
    indexed_points.push_back({key, static_cast<uint32_t>(i), point.z()});
  }

  if (indexed_points.empty()) {
    cloud.clear();
    return std::move(cloud);
  }

  // Step 2: Sort by 2D grid key
  std::sort(indexed_points.begin(), indexed_points.end());

  // Step 3: Collect indices of max-Z points
  std::vector<uint32_t> max_z_indices;
  max_z_indices.reserve(indexed_points.size() / 4 + 100);

  size_t group_start = 0;
  while (group_start < indexed_points.size()) {
    uint64_t current_key = indexed_points[group_start].key;

    // Find point with max Z in this cell
    uint32_t max_z_idx = indexed_points[group_start].index;
    float max_z = indexed_points[group_start].z;

    size_t group_end = group_start + 1;
    while (group_end < indexed_points.size() &&
           indexed_points[group_end].key == current_key) {
      if (indexed_points[group_end].z > max_z) {
        max_z = indexed_points[group_end].z;
        max_z_idx = indexed_points[group_end].index;
      }
      ++group_end;
    }

    max_z_indices.push_back(max_z_idx);
    group_start = group_end;
  }

  // Step 4: Build output cloud (cannot do true in-place due to index ordering)
  PointCloud output;
  output.setTimestamp(cloud.timestamp());
  output.setFrameId(cloud.frameId());
  output.reserve(max_z_indices.size());

  if (has_intensity) output.enableIntensity();
  if (has_time) output.enableTime();
  if (has_ring) output.enableRing();
  if (has_color) output.enableColor();
  if (has_label) output.enableLabel();

  for (uint32_t idx : max_z_indices) {
    output.add(cloud[idx]);
    detail::copyAttributes(cloud, idx, output, has_intensity, has_time,
                           has_ring, has_color, has_label);
  }

  return output;
}

}  // namespace filters
}  // namespace npcl

#endif  // NANOPCL_FILTERS_IMPL_GRID_MAX_Z_IMPL_HPP
