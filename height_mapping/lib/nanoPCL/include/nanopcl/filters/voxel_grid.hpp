// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_VOXEL_GRID_HPP
#define NANOPCL_FILTERS_VOXEL_GRID_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace filters {

/**
 * @brief High-performance voxel grid downsampling filter
 *
 * Uses sorting-based algorithm for cache-friendly, SIMD-optimizable processing.
 * Much faster than map-based approaches for large point clouds.
 *
 * Algorithm:
 * 1. Compute 64-bit voxel key for each point
 * 2. Sort points by voxel key (cache-friendly sequential access)
 * 3. Iterate through sorted array, compute representative for each voxel group
 *
 * Usage:
 * @code
 * VoxelGrid filter({.voxel_size = 0.1f});
 * filter.filterInPlace(cloud);
 * @endcode
 */
class VoxelGrid {
 public:
  enum class Method {
    CENTROID,     // Average of all points in voxel
    RANDOM,       // Random point from voxel
    FIRST,        // First point added to voxel
    CLOSEST,      // Point closest to voxel center
    VOXEL_CENTER  // Voxel center itself
  };

  enum class IntensityMethod {
    AVERAGE,  // Average of all intensities in voxel (default)
    MAX       // Maximum intensity in voxel (useful for reflector detection)
  };

  struct Config {
    float voxel_size = 0.1f;
    Method method = Method::CENTROID;
    IntensityMethod intensity_method = IntensityMethod::AVERAGE;

    void validate() const {
      if (voxel_size <= 0 || voxel_size > 10.0f) {
        throw std::invalid_argument("voxel_size must be in (0, 10]");
      }
    }
  };

  struct Stats {
    size_t input_size = 0;
    size_t output_size = 0;
    size_t voxel_count = 0;
  };

 private:
  Config config_;

  // Precomputed inverse for faster division
  float inv_voxel_size_ = 10.0f;

  // Packed voxel key with original index
  struct IndexedPoint {
    uint64_t key;
    uint32_t index;

    bool operator<(const IndexedPoint& other) const { return key < other.key; }
  };

 public:
  VoxelGrid() {
    config_.validate();
    inv_voxel_size_ = 1.0f / config_.voxel_size;
  }

  explicit VoxelGrid(const Config& config) : config_(config) {
    config_.validate();
    inv_voxel_size_ = 1.0f / config_.voxel_size;
  }

  void setConfig(const Config& config) {
    config_ = config;
    config_.validate();
    inv_voxel_size_ = 1.0f / config_.voxel_size;
  }

  const Config& config() const { return config_; }

  /**
   * @brief Apply voxel filter in-place (sorting-based, high performance)
   */
  Stats filterInPlace(PointCloud& cloud) const {
    Stats stats;
    stats.input_size = cloud.size();

    if (cloud.empty()) {
      return stats;
    }

    // Cache channel presence
    const bool has_intensity = cloud.hasIntensity();
    const bool has_time = cloud.hasTime();
    const bool has_ring = cloud.hasRing();
    const bool has_color = cloud.hasColor();
    const bool has_label = cloud.hasLabel();

    // Step 1: Compute voxel keys for all points
    std::vector<IndexedPoint> indexed_points;
    indexed_points.reserve(cloud.size());

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      const auto& point = cloud.points[i];
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
          !std::isfinite(point.z())) {
        continue;
      }
      indexed_points.push_back(
          {computeVoxelKey(point), static_cast<uint32_t>(i)});
    }

    if (indexed_points.empty()) {
      cloud.clear();
      return stats;
    }

    // Step 2: Sort by voxel key (cache-friendly operation)
    std::sort(indexed_points.begin(), indexed_points.end());

    // Step 3: Create output cloud
    PointCloud filtered;
    filtered.reserve(indexed_points.size() / 4);  // Rough estimate

    if (has_intensity) filtered.enableIntensity();
    if (has_time) filtered.enableTime();
    if (has_ring) filtered.enableRing();
    if (has_color) filtered.enableColor();
    if (has_label) filtered.enableLabel();

    // Step 4: Process each voxel group
    size_t group_start = 0;
    while (group_start < indexed_points.size()) {
      uint64_t current_key = indexed_points[group_start].key;

      // Find end of current voxel group
      size_t group_end = group_start + 1;
      while (group_end < indexed_points.size() &&
             indexed_points[group_end].key == current_key) {
        ++group_end;
      }

      // Compute representative point for this voxel
      addRepresentativePoint(cloud, indexed_points, group_start, group_end,
                             filtered, has_intensity, has_time, has_ring,
                             has_color, has_label);

      group_start = group_end;
      ++stats.voxel_count;
    }

    stats.output_size = filtered.size();

    // Preserve metadata
    filtered.setTimestamp(cloud.timestamp());
    filtered.setFrameId(cloud.frameId());

    cloud = std::move(filtered);
    return stats;
  }

  /**
   * @brief Apply voxel filter to copy
   */
  PointCloud filter(const PointCloud& cloud) const {
    PointCloud result = cloud;
    filterInPlace(result);
    return result;
  }

 private:
  /**
   * @brief Compute 64-bit voxel key from 3D point
   *
   * Packs (x, y, z) voxel indices into a single 64-bit integer.
   * Each dimension uses 21 bits, allowing range of [-1M, +1M] voxels.
   */
  uint64_t computeVoxelKey(const Point& p) const {
    // Use multiplication instead of division (faster)
    int32_t ix = static_cast<int32_t>(std::floor(p.x() * inv_voxel_size_));
    int32_t iy = static_cast<int32_t>(std::floor(p.y() * inv_voxel_size_));
    int32_t iz = static_cast<int32_t>(std::floor(p.z() * inv_voxel_size_));

    // Offset to handle negative indices (shift to positive range)
    constexpr int32_t offset = 1 << 20;  // 1M offset
    uint64_t ux = static_cast<uint64_t>(ix + offset);
    uint64_t uy = static_cast<uint64_t>(iy + offset);
    uint64_t uz = static_cast<uint64_t>(iz + offset);

    // Pack into 64-bit key (21 bits each)
    return (ux << 42) | (uy << 21) | uz;
  }

  /**
   * @brief Extract voxel center from key (for VOXEL_CENTER and CLOSEST methods)
   */
  Point keyToVoxelCenter(uint64_t key) const {
    constexpr int32_t offset = 1 << 20;

    int32_t ix = static_cast<int32_t>((key >> 42) & 0x1FFFFF) - offset;
    int32_t iy = static_cast<int32_t>((key >> 21) & 0x1FFFFF) - offset;
    int32_t iz = static_cast<int32_t>(key & 0x1FFFFF) - offset;

    return Point((ix + 0.5f) * config_.voxel_size,
                 (iy + 0.5f) * config_.voxel_size,
                 (iz + 0.5f) * config_.voxel_size);
  }

  /**
   * @brief Add representative point for a voxel group to output cloud
   */
  void addRepresentativePoint(const PointCloud& input,
                              const std::vector<IndexedPoint>& indexed_points,
                              size_t group_start, size_t group_end,
                              PointCloud& output, bool has_intensity,
                              bool has_time, bool has_ring, bool has_color,
                              bool has_label) const {
    const size_t group_size = group_end - group_start;

    switch (config_.method) {
      case Method::CENTROID: {
        // Compute centroid of all points in voxel
        Point centroid = Point::Zero();
        for (size_t i = group_start; i < group_end; ++i) {
          centroid += input.points[indexed_points[i].index];
        }
        output.points.push_back(centroid / static_cast<float>(group_size));

        // Add averaged attributes
        addAveragedAttributes(input, indexed_points, group_start, group_end,
                              output, has_intensity, has_time, has_ring,
                              has_color, has_label);
        break;
      }

      case Method::FIRST: {
        uint32_t idx = indexed_points[group_start].index;
        output.points.push_back(input.points[idx]);
        copyAttributes(input, idx, output, has_intensity, has_time, has_ring,
                       has_color, has_label);
        break;
      }

      case Method::RANDOM: {
        // Use deterministic "random" based on group properties
        size_t random_offset = (group_size * 7 + group_start * 13) % group_size;
        uint32_t idx = indexed_points[group_start + random_offset].index;
        output.points.push_back(input.points[idx]);
        copyAttributes(input, idx, output, has_intensity, has_time, has_ring,
                       has_color, has_label);
        break;
      }

      case Method::CLOSEST: {
        Point center = keyToVoxelCenter(indexed_points[group_start].key);
        float min_dist_sq = std::numeric_limits<float>::max();
        uint32_t closest_idx = indexed_points[group_start].index;

        for (size_t i = group_start; i < group_end; ++i) {
          uint32_t idx = indexed_points[i].index;
          float dist_sq = (input.points[idx] - center).squaredNorm();
          if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = idx;
          }
        }
        output.points.push_back(input.points[closest_idx]);
        copyAttributes(input, closest_idx, output, has_intensity, has_time,
                       has_ring, has_color, has_label);
        break;
      }

      case Method::VOXEL_CENTER: {
        output.points.push_back(
            keyToVoxelCenter(indexed_points[group_start].key));
        addAveragedAttributes(input, indexed_points, group_start, group_end,
                              output, has_intensity, has_time, has_ring,
                              has_color, has_label);
        break;
      }
    }
  }

  /**
   * @brief Copy attributes from single point
   */
  void copyAttributes(const PointCloud& input, uint32_t idx, PointCloud& output,
                      bool has_intensity, bool has_time, bool has_ring,
                      bool has_color, bool has_label) const {
    if (has_intensity) output.intensity().push_back(input.intensity()[idx]);
    if (has_time) output.time().push_back(input.time()[idx]);
    if (has_ring) output.ring().push_back(input.ring()[idx]);
    if (has_color) output.color().push_back(input.color()[idx]);
    if (has_label) output.label().push_back(input.label()[idx]);
  }

  /**
   * @brief Add averaged attributes for voxel group
   */
  void addAveragedAttributes(const PointCloud& input,
                             const std::vector<IndexedPoint>& indexed_points,
                             size_t group_start, size_t group_end,
                             PointCloud& output, bool has_intensity,
                             bool has_time, bool has_ring, bool has_color,
                             bool has_label) const {
    const float n = static_cast<float>(group_end - group_start);

    if (has_intensity) {
      if (config_.intensity_method == IntensityMethod::MAX) {
        float max_intensity =
            input.intensity()[indexed_points[group_start].index];
        for (size_t i = group_start + 1; i < group_end; ++i) {
          float val = input.intensity()[indexed_points[i].index];
          if (val > max_intensity) max_intensity = val;
        }
        output.intensity().push_back(max_intensity);
      } else {
        float sum = 0.0f;
        for (size_t i = group_start; i < group_end; ++i) {
          sum += input.intensity()[indexed_points[i].index];
        }
        output.intensity().push_back(sum / n);
      }
    }

    if (has_time) {
      float sum = 0.0f;
      for (size_t i = group_start; i < group_end; ++i) {
        sum += input.time()[indexed_points[i].index];
      }
      output.time().push_back(sum / n);
    }

    if (has_ring) {
      // Ring: pick first (averaging ring IDs doesn't make sense)
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
      // Label: pick first (semantic labels are discrete values)
      output.label().push_back(
          input.label()[indexed_points[group_start].index]);
    }
  }
};

}  // namespace filters
}  // namespace nanopcl

#endif  // NANOPCL_FILTERS_VOXEL_GRID_HPP
