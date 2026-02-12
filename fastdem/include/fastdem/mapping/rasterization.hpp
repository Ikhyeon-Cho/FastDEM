// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * rasterization.hpp
 *
 * Point cloud to grid rasterization (binning).
 * Converts 3D points to 2D grid cells, preserving all channels.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_MAPPING_RASTERIZATION_HPP
#define FASTDEM_MAPPING_RASTERIZATION_HPP

#include <memory>
#include <unordered_map>
#include <vector>

#include "fastdem/config/rasterization.hpp"
#include "fastdem/elevation_map.hpp"
#include "fastdem/point_types.hpp"

namespace fastdem {

namespace detail {

// Hash function for grid_map::Index
struct IndexHash {
  std::size_t operator()(const grid_map::Index& idx) const {
    return std::hash<int>()(idx(0)) ^ (std::hash<int>()(idx(1)) << 16);
  }
};

// Equality for grid_map::Index
struct IndexEqual {
  bool operator()(const grid_map::Index& a, const grid_map::Index& b) const {
    return a(0) == b(0) && a(1) == b(1);
  }
};

// Accumulator for cell statistics
struct CellAccumulator {
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  size_t max_idx = 0;
  size_t min_idx = 0;
  size_t count = 0;
  // For Mean aggregation
  float sum_z = 0.0f;
  std::vector<size_t> point_indices;
};

}  // namespace detail

/**
 * @brief Point cloud to grid rasterization.
 *
 * Bins 3D points into 2D grid cells and selects representative points.
 * All channels (intensity, color, covariance, etc.) are preserved.
 *
 * @code
 *   Rasterization rasterizer(RasterMethod::Max);
 *   PointCloud rasterized = rasterizer.process(cloud, map);
 * @endcode
 */
class Rasterization {
 public:
  using Config = config::Rasterization;

  Rasterization() : cfg_{} {}
  explicit Rasterization(const Config& cfg) : cfg_{cfg} {}
  explicit Rasterization(RasterMethod method) : cfg_{method} {}

  /**
   * @brief Rasterize point cloud to grid cells.
   *
   * @param cloud Point cloud in map frame (with all channels)
   * @param map ElevationMap (provides grid geometry for bounds checking)
   * @return Rasterized PointCloud with one point per cell, all channels
   * preserved
   */
  PointCloud process(const PointCloud& cloud, const ElevationMap& map) const;

  RasterMethod method() const { return cfg_.method; }

 private:
  Config cfg_;
};

inline PointCloud Rasterization::process(const PointCloud& cloud,
                                         const ElevationMap& map) const {
  if (cloud.empty()) return {};

  using namespace detail;
  std::unordered_map<grid_map::Index, CellAccumulator, IndexHash, IndexEqual>
      cells;

  // First pass: group points by cell and track statistics
  for (size_t i : cloud.indices()) {
    auto pt = cloud.point(i);
    grid_map::Index index;
    if (!map.getIndex(grid_map::Position(pt.x(), pt.y()), index)) {
      continue;  // Outside map bounds
    }

    auto& acc = cells[index];
    const float z = pt.z();

    if (z > acc.max_z) {
      acc.max_z = z;
      acc.max_idx = i;
    }
    if (z < acc.min_z) {
      acc.min_z = z;
      acc.min_idx = i;
    }
    acc.sum_z += z;
    acc.count++;

    if (cfg_.method == RasterMethod::Mean) {
      acc.point_indices.push_back(i);
    }
  }

  if (cells.empty()) return {};

  // Second pass: extract representative points
  if (cfg_.method == RasterMethod::Max || cfg_.method == RasterMethod::Min) {
    // Max/Min: extract representative point indices
    std::vector<size_t> indices;
    indices.reserve(cells.size());

    for (const auto& [index, acc] : cells) {
      indices.push_back(cfg_.method == RasterMethod::Max ? acc.max_idx
                                                         : acc.min_idx);
    }

    return cloud.extract(indices);
  }

  // Mean: compute averaged values for each channel
  PointCloud result;
  result.copyChannelLayout(cloud);
  result.resize(cells.size());

  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();
  const bool has_covariance = cloud.hasCovariance();

  size_t out_idx = 0;
  for (const auto& [index, acc] : cells) {
    // Mean position
    float mean_z = acc.sum_z / static_cast<float>(acc.count);

    // Use first point's XY (they're all in same cell)
    auto first_pt = cloud.point(acc.point_indices[0]);
    result.point(out_idx) = Eigen::Vector3f(first_pt.x(), first_pt.y(), mean_z);

    // Mean channels
    if (has_intensity) {
      float sum = 0.0f;
      for (size_t idx : acc.point_indices) {
        sum += cloud.intensity(idx);
      }
      result.intensity(out_idx) = sum / static_cast<float>(acc.count);
    }

    if (has_color) {
      float sum_r = 0.0f, sum_g = 0.0f, sum_b = 0.0f;
      for (size_t idx : acc.point_indices) {
        const auto& c = cloud.color(idx);
        sum_r += c.r;
        sum_g += c.g;
        sum_b += c.b;
      }
      float n = static_cast<float>(acc.count);
      result.color(out_idx) =
          nanopcl::Color{static_cast<uint8_t>(sum_r / n + 0.5f),
                         static_cast<uint8_t>(sum_g / n + 0.5f),
                         static_cast<uint8_t>(sum_b / n + 0.5f)};
    }

    if (has_covariance) {
      Eigen::Matrix3f sum_cov = Eigen::Matrix3f::Zero();
      for (size_t idx : acc.point_indices) {
        sum_cov += cloud.covariance(idx);
      }
      result.covariance(out_idx) = sum_cov / static_cast<float>(acc.count);
    }

    out_idx++;
  }

  return result;
}

/// Factory function for consistent creation pattern
inline std::unique_ptr<Rasterization> createRasterization(
    const config::Rasterization& cfg) {
  return std::make_unique<Rasterization>(cfg);
}

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_RASTERIZATION_HPP
