// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for voxel grid filter.
// Do not include this file directly; include <nanopcl/filters/downsample.hpp>

#ifndef NANOPCL_FILTERS_IMPL_VOXEL_GRID_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_VOXEL_GRID_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace npcl {
namespace filters {
namespace detail {

// Voxel coordinate limits (21 bits per axis, with offset)
// Valid range after offset: [0, 2^21 - 1] = [0, 2097151]
// With offset of 2^20, original range: [-1048576, 1048575]
// At voxel_size=0.1m, this covers ~±104km which is sufficient for robotics
constexpr int32_t VOXEL_COORD_OFFSET = 1 << 20;  // 1048576
constexpr int32_t VOXEL_COORD_MIN = -VOXEL_COORD_OFFSET;
constexpr int32_t VOXEL_COORD_MAX = VOXEL_COORD_OFFSET - 1;
constexpr uint64_t VOXEL_COORD_MASK = 0x1FFFFF;  // 21 bits

/// @brief Minimum supported voxel size (1mm) to ensure spatial range safety
constexpr float MIN_VOXEL_SIZE = 0.001f;

// Packed voxel key with original index
struct IndexedPoint {
  uint64_t key;
  uint32_t index;
  bool operator<(const IndexedPoint& other) const { return key < other.key; }
};

inline uint64_t computeVoxelKey(const Point& p, float inv_voxel_size) {
  int32_t ix = static_cast<int32_t>(std::floor(p.x() * inv_voxel_size));
  int32_t iy = static_cast<int32_t>(std::floor(p.y() * inv_voxel_size));
  int32_t iz = static_cast<int32_t>(std::floor(p.z() * inv_voxel_size));

  // Clamp to valid range (21 bits per coordinate)
  ix = std::clamp(ix, VOXEL_COORD_MIN, VOXEL_COORD_MAX);
  iy = std::clamp(iy, VOXEL_COORD_MIN, VOXEL_COORD_MAX);
  iz = std::clamp(iz, VOXEL_COORD_MIN, VOXEL_COORD_MAX);

  // Offset to handle negative indices
  uint64_t ux = static_cast<uint64_t>(ix + VOXEL_COORD_OFFSET);
  uint64_t uy = static_cast<uint64_t>(iy + VOXEL_COORD_OFFSET);
  uint64_t uz = static_cast<uint64_t>(iz + VOXEL_COORD_OFFSET);

  // Pack into 64-bit key (21 bits each)
  return (ux << 42) | (uy << 21) | uz;
}

inline Point keyToVoxelCenter(uint64_t key, float voxel_size) {
  int32_t ix =
      static_cast<int32_t>((key >> 42) & VOXEL_COORD_MASK) - VOXEL_COORD_OFFSET;
  int32_t iy =
      static_cast<int32_t>((key >> 21) & VOXEL_COORD_MASK) - VOXEL_COORD_OFFSET;
  int32_t iz =
      static_cast<int32_t>(key & VOXEL_COORD_MASK) - VOXEL_COORD_OFFSET;

  return Point((ix + 0.5f) * voxel_size, (iy + 0.5f) * voxel_size,
               (iz + 0.5f) * voxel_size);
}

inline void copyAttributes(const PointCloud& input, uint32_t idx,
                           PointCloud& output, bool has_intensity,
                           bool has_time, bool has_ring, bool has_color,
                           bool has_label) {
  if (has_intensity) output.intensity().push_back(input.intensity()[idx]);
  if (has_time) output.time().push_back(input.time()[idx]);
  if (has_ring) output.ring().push_back(input.ring()[idx]);
  if (has_color) output.color().push_back(input.color()[idx]);
  if (has_label) output.label().push_back(input.label()[idx]);
}

inline void addAveragedAttributes(
    const PointCloud& input, const std::vector<IndexedPoint>& indexed_points,
    size_t group_start, size_t group_end, PointCloud& output,
    bool has_intensity, bool has_time, bool has_ring, bool has_color,
    bool has_label) {
  const float n = static_cast<float>(group_end - group_start);

  if (has_intensity) {
    float sum = 0.0f;
    for (size_t i = group_start; i < group_end; ++i) {
      sum += input.intensity()[indexed_points[i].index];
    }
    output.intensity().push_back(sum / n);
  }

  if (has_time) {
    float sum = 0.0f;
    for (size_t i = group_start; i < group_end; ++i) {
      sum += input.time()[indexed_points[i].index];
    }
    output.time().push_back(sum / n);
  }

  if (has_ring) {
    output.ring().push_back(input.ring()[indexed_points[group_start].index]);
  }

  if (has_color) {
    float r = 0, g = 0, b = 0;
    for (size_t i = group_start; i < group_end; ++i) {
      const auto& c = input.color()[indexed_points[i].index];
      r += c[0];
      g += c[1];
      b += c[2];
    }
    output.color().push_back({static_cast<uint8_t>(r / n),
                              static_cast<uint8_t>(g / n),
                              static_cast<uint8_t>(b / n)});
  }

  if (has_label) {
    output.label().push_back(input.label()[indexed_points[group_start].index]);
  }
}

inline void addRepresentativePoint(
    const PointCloud& input, const std::vector<IndexedPoint>& indexed_points,
    size_t group_start, size_t group_end, PointCloud& output,
    VoxelMethod method, float voxel_size, bool has_intensity, bool has_time,
    bool has_ring, bool has_color, bool has_label) {
  const size_t group_size = group_end - group_start;

  switch (method) {
    case VoxelMethod::CENTROID: {
      Point centroid = Point::Zero();
      for (size_t i = group_start; i < group_end; ++i) {
        centroid += input[indexed_points[i].index];
      }
      output.xyz().push_back(centroid / static_cast<float>(group_size));
      addAveragedAttributes(input, indexed_points, group_start, group_end,
                            output, has_intensity, has_time, has_ring,
                            has_color, has_label);
      break;
    }

    case VoxelMethod::FIRST: {
      uint32_t idx = indexed_points[group_start].index;
      output.xyz().push_back(input[idx]);
      copyAttributes(input, idx, output, has_intensity, has_time, has_ring,
                     has_color, has_label);
      break;
    }

    case VoxelMethod::ARBITRARY: {
      // Deterministic selection (reproducible, not truly random)
      size_t offset = (group_size * 7 + group_start * 13) % group_size;
      uint32_t idx = indexed_points[group_start + offset].index;
      output.xyz().push_back(input[idx]);
      copyAttributes(input, idx, output, has_intensity, has_time, has_ring,
                     has_color, has_label);
      break;
    }

    case VoxelMethod::CLOSEST: {
      Point center =
          keyToVoxelCenter(indexed_points[group_start].key, voxel_size);
      float min_dist_sq = std::numeric_limits<float>::max();
      uint32_t closest_idx = indexed_points[group_start].index;

      for (size_t i = group_start; i < group_end; ++i) {
        uint32_t idx = indexed_points[i].index;
        float dist_sq = (input[idx] - center).squaredNorm();
        if (dist_sq < min_dist_sq) {
          min_dist_sq = dist_sq;
          closest_idx = idx;
        }
      }
      output.xyz().push_back(input[closest_idx]);
      copyAttributes(input, closest_idx, output, has_intensity, has_time,
                     has_ring, has_color, has_label);
      break;
    }

    case VoxelMethod::VOXEL_CENTER: {
      output.xyz().push_back(keyToVoxelCenter(indexed_points[group_start].key, voxel_size));
      addAveragedAttributes(input, indexed_points, group_start, group_end,
                            output, has_intensity, has_time, has_ring,
                            has_color, has_label);
      break;
    }
  }
}

// Thread-local buffer for indexed points (avoids repeated allocation)
// Benchmarked: 1.2x-1.6x faster than per-call allocation
inline std::vector<IndexedPoint>& getIndexedPointsBuffer() {
  thread_local std::vector<IndexedPoint> buffer;
  return buffer;
}

// Thread-local buffer for voxel results (used by move version)
struct VoxelResult {
  Point centroid;
  float intensity;
  float time;
  uint16_t ring;
  Color color;
  Label label;
};

inline std::vector<VoxelResult>& getResultsBuffer() {
  thread_local std::vector<VoxelResult> buffer;
  return buffer;
}

}  // namespace detail

inline PointCloud voxelGrid(const PointCloud& cloud, float voxel_size) {
  return voxelGrid(cloud, voxel_size, VoxelMethod::CENTROID);
}

inline PointCloud voxelGrid(const PointCloud& cloud, float voxel_size,
                            VoxelMethod method) {
  if (voxel_size < detail::MIN_VOXEL_SIZE || voxel_size > 100.0f) {
    throw std::invalid_argument(
        "voxel_size must be in [0.001, 100]. "
        "Spatial range is limited to +/- 1,048,576 * voxel_size meters.");
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

  // Step 1: Compute voxel keys (using TLS buffer)
  const float inv_voxel_size = 1.0f / voxel_size;
  auto& indexed_points = detail::getIndexedPointsBuffer();
  indexed_points.clear();
  indexed_points.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }
    uint64_t key = detail::computeVoxelKey(point, inv_voxel_size);
    indexed_points.push_back({key, static_cast<uint32_t>(i)});
  }

  if (indexed_points.empty()) {
    return output;
  }

  // Step 2: Sort by voxel key
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

  // Step 4: Process voxel groups
  size_t group_start = 0;
  while (group_start < indexed_points.size()) {
    uint64_t current_key = indexed_points[group_start].key;

    // Find end of current voxel group
    size_t group_end = group_start + 1;
    while (group_end < indexed_points.size() &&
           indexed_points[group_end].key == current_key) {
      ++group_end;
    }

    detail::addRepresentativePoint(
        cloud, indexed_points, group_start, group_end, output, method,
        voxel_size, has_intensity, has_time, has_ring, has_color, has_label);

    group_start = group_end;
  }

  return output;
}

// =============================================================================
// Move versions (reuse input buffer for optimal performance)
// =============================================================================

inline PointCloud voxelGrid(PointCloud&& cloud, float voxel_size) {
  return voxelGrid(std::move(cloud), voxel_size, VoxelMethod::CENTROID);
}

inline PointCloud voxelGrid(PointCloud&& cloud, float voxel_size,
                            VoxelMethod method) {
  if (voxel_size < detail::MIN_VOXEL_SIZE || voxel_size > 100.0f) {
    throw std::invalid_argument(
        "voxel_size must be in [0.001, 100]. "
        "Spatial range is limited to +/- 1,048,576 * voxel_size meters.");
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

  // Step 1: Compute voxel keys (using TLS buffer)
  const float inv_voxel_size = 1.0f / voxel_size;
  auto& indexed_points = detail::getIndexedPointsBuffer();
  indexed_points.clear();
  indexed_points.reserve(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& point = cloud[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }
    uint64_t key = detail::computeVoxelKey(point, inv_voxel_size);
    indexed_points.push_back({key, static_cast<uint32_t>(i)});
  }

  if (indexed_points.empty()) {
    cloud.clear();
    return std::move(cloud);
  }

  // Step 2: Sort by voxel key
  std::sort(indexed_points.begin(), indexed_points.end());

  // Step 3: Compute all results first (using TLS buffer)
  auto& results = detail::getResultsBuffer();
  results.clear();
  results.reserve(indexed_points.size() / 4 + 100);

  size_t group_start = 0;
  while (group_start < indexed_points.size()) {
    uint64_t current_key = indexed_points[group_start].key;

    size_t group_end = group_start + 1;
    while (group_end < indexed_points.size() &&
           indexed_points[group_end].key == current_key) {
      ++group_end;
    }

    const size_t group_size = group_end - group_start;
    const float n = static_cast<float>(group_size);
    detail::VoxelResult res{};

    switch (method) {
      case VoxelMethod::CENTROID: {
        Point centroid = Point::Zero();
        float sum_intensity = 0, sum_time = 0;
        float sum_r = 0, sum_g = 0, sum_b = 0;

        for (size_t i = group_start; i < group_end; ++i) {
          uint32_t idx = indexed_points[i].index;
          centroid += cloud[idx];
          if (has_intensity) sum_intensity += cloud.intensity()[idx];
          if (has_time) sum_time += cloud.time()[idx];
          if (has_color) {
            sum_r += cloud.color()[idx][0];
            sum_g += cloud.color()[idx][1];
            sum_b += cloud.color()[idx][2];
          }
        }

        res.centroid = centroid / n;
        res.intensity = sum_intensity / n;
        res.time = sum_time / n;
        res.ring =
            has_ring ? cloud.ring()[indexed_points[group_start].index] : 0;
        res.color = has_color ? Color(static_cast<uint8_t>(sum_r / n),
                                      static_cast<uint8_t>(sum_g / n),
                                      static_cast<uint8_t>(sum_b / n))
                              : Color();
        res.label = has_label ? cloud.label()[indexed_points[group_start].index]
                              : Label();
        break;
      }

      case VoxelMethod::FIRST: {
        uint32_t idx = indexed_points[group_start].index;
        res.centroid = cloud[idx];
        res.intensity = has_intensity ? cloud.intensity()[idx] : 0;
        res.time = has_time ? cloud.time()[idx] : 0;
        res.ring = has_ring ? cloud.ring()[idx] : 0;
        res.color = has_color ? cloud.color()[idx] : Color();
        res.label = has_label ? cloud.label()[idx] : Label();
        break;
      }

      case VoxelMethod::ARBITRARY: {
        size_t offset = (group_size * 7 + group_start * 13) % group_size;
        uint32_t idx = indexed_points[group_start + offset].index;
        res.centroid = cloud[idx];
        res.intensity = has_intensity ? cloud.intensity()[idx] : 0;
        res.time = has_time ? cloud.time()[idx] : 0;
        res.ring = has_ring ? cloud.ring()[idx] : 0;
        res.color = has_color ? cloud.color()[idx] : Color();
        res.label = has_label ? cloud.label()[idx] : Label();
        break;
      }

      case VoxelMethod::CLOSEST: {
        Point center = detail::keyToVoxelCenter(indexed_points[group_start].key,
                                                voxel_size);
        float min_dist_sq = std::numeric_limits<float>::max();
        uint32_t closest_idx = indexed_points[group_start].index;

        for (size_t i = group_start; i < group_end; ++i) {
          uint32_t idx = indexed_points[i].index;
          float dist_sq = (cloud[idx] - center).squaredNorm();
          if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = idx;
          }
        }

        res.centroid = cloud[closest_idx];
        res.intensity = has_intensity ? cloud.intensity()[closest_idx] : 0;
        res.time = has_time ? cloud.time()[closest_idx] : 0;
        res.ring = has_ring ? cloud.ring()[closest_idx] : 0;
        res.color = has_color ? cloud.color()[closest_idx] : Color();
        res.label = has_label ? cloud.label()[closest_idx] : Label();
        break;
      }

      case VoxelMethod::VOXEL_CENTER: {
        float sum_intensity = 0, sum_time = 0;
        float sum_r = 0, sum_g = 0, sum_b = 0;

        for (size_t i = group_start; i < group_end; ++i) {
          uint32_t idx = indexed_points[i].index;
          if (has_intensity) sum_intensity += cloud.intensity()[idx];
          if (has_time) sum_time += cloud.time()[idx];
          if (has_color) {
            sum_r += cloud.color()[idx][0];
            sum_g += cloud.color()[idx][1];
            sum_b += cloud.color()[idx][2];
          }
        }

        res.centroid = detail::keyToVoxelCenter(indexed_points[group_start].key,
                                                voxel_size);
        res.intensity = sum_intensity / n;
        res.time = sum_time / n;
        res.ring =
            has_ring ? cloud.ring()[indexed_points[group_start].index] : 0;
        res.color = has_color ? Color(static_cast<uint8_t>(sum_r / n),
                                      static_cast<uint8_t>(sum_g / n),
                                      static_cast<uint8_t>(sum_b / n))
                              : Color();
        res.label = has_label ? cloud.label()[indexed_points[group_start].index]
                              : Label();
        break;
      }
    }

    results.push_back(res);
    group_start = group_end;
  }

  // Step 4: Write results to input cloud's buffer (reuse memory)
  cloud.resize(results.size());
  Point* pts = cloud.xyz().data();
  float* intensity = has_intensity ? cloud.intensity().data() : nullptr;
  float* time = has_time ? cloud.time().data() : nullptr;
  uint16_t* ring = has_ring ? cloud.ring().data() : nullptr;
  Color* color = has_color ? cloud.color().data() : nullptr;
  Label* label = has_label ? cloud.label().data() : nullptr;

  for (size_t i = 0; i < results.size(); ++i) {
    pts[i] = results[i].centroid;
    if (intensity) intensity[i] = results[i].intensity;
    if (time) time[i] = results[i].time;
    if (ring) ring[i] = results[i].ring;
    if (color) color[i] = results[i].color;
    if (label) label[i] = results[i].label;
  }

  return std::move(cloud);
}

}  // namespace filters
}  // namespace npcl

#endif  // NANOPCL_FILTERS_IMPL_VOXEL_GRID_IMPL_HPP
