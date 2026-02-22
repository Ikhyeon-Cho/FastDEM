// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * pcd_convert.cpp
 *
 * Conversions between PointCloud and ElevationMap.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/io/pcd_convert.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "fastdem/mapping/grid_index_hash.hpp"
#include "fastdem/postprocess/inpainting.hpp"
#include "nanopcl/filters/core.hpp"
#include "nanopcl/filters/outlier_removal.hpp"

namespace fastdem {

// ─── fromPointCloud ─────────────────────────────────────────────────────

namespace {

/// Per-cell statistics accumulated from all points.
struct BatchCellStats {
  float mean = 0.0f;
  float m2 = 0.0f;
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  uint32_t count = 0;
  float max_intensity = std::numeric_limits<float>::lowest();
  float last_color_packed = 0.0f;
  bool has_intensity = false;
  bool has_color = false;

  /// Welford online update
  void addZ(float z) {
    ++count;
    const float delta = z - mean;
    mean += delta / static_cast<float>(count);
    const float delta2 = z - mean;
    m2 += delta * delta2;

    if (z < min_z) min_z = z;
    if (z > max_z) max_z = z;
  }

  /// Sample variance (0 for single point)
  float variance() const {
    return (count < 2) ? 0.0f : m2 / static_cast<float>(count - 1);
  }
};

}  // namespace

void fromPointCloud(const PointCloud& cloud, ElevationMap& map,
                    RasterMethod method) {
  if (cloud.empty()) return;

  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();

  // ── Phase 1: Hash all points into cells, accumulate statistics ──
  CellMap<BatchCellStats> cells;
  cells.reserve(cloud.size() / 4);  // Rough estimate of unique cells

  for (size_t i : cloud.indices()) {
    auto pt = cloud.point(i);
    const float z = pt.z();
    if (std::isnan(z)) continue;

    grid_map::Index index;
    if (!map.getIndex(grid_map::Position(pt.x(), pt.y()), index)) continue;

    auto& stats = cells[index];
    stats.addZ(z);

    if (has_intensity) {
      float val = cloud.intensity(i);
      if (!stats.has_intensity || val > stats.max_intensity) {
        stats.max_intensity = val;
        stats.has_intensity = true;
      }
    }

    if (has_color) {
      auto color = cloud.color(i);
      Eigen::Vector3i rgb(color.r, color.g, color.b);
      float packed;
      grid_map::colorVectorToValue(rgb, packed);
      stats.last_color_packed = packed;
      stats.has_color = true;
    }
  }

  if (cells.empty()) return;

  // ── Phase 2: Ensure layers exist ──
  if (!map.exists(layer::elevation_min)) map.add(layer::elevation_min, NAN);
  if (!map.exists(layer::elevation_max)) map.add(layer::elevation_max, NAN);
  if (!map.exists(layer::variance)) map.add(layer::variance, NAN);
  if (!map.exists(layer::n_points)) map.add(layer::n_points, 0.0f);
  if (has_intensity && !map.exists(layer::intensity))
    map.add(layer::intensity, NAN);
  if (has_color && !map.exists(layer::color)) map.add(layer::color, NAN);

  auto& elev_mat = map.get(layer::elevation);
  auto& min_mat = map.get(layer::elevation_min);
  auto& max_mat = map.get(layer::elevation_max);
  auto& var_mat = map.get(layer::variance);
  auto& count_mat = map.get(layer::n_points);

  // ── Phase 3: Write to map layers ──
  for (const auto& [index, stats] : cells) {
    const int i = index(0);
    const int j = index(1);

    switch (method) {
      case RasterMethod::Max:
        elev_mat(i, j) = stats.max_z;
        break;
      case RasterMethod::Min:
        elev_mat(i, j) = stats.min_z;
        break;
      case RasterMethod::Mean:
        elev_mat(i, j) = stats.mean;
        break;
      case RasterMethod::MinMax:
        elev_mat(i, j) = stats.max_z;
        break;
    }

    min_mat(i, j) = stats.min_z;
    max_mat(i, j) = stats.max_z;
    var_mat(i, j) = stats.variance();
    count_mat(i, j) = static_cast<float>(stats.count);

    if (has_intensity && stats.has_intensity) {
      map.get(layer::intensity)(i, j) = stats.max_intensity;
    }

    if (has_color && stats.has_color) {
      map.get(layer::color)(i, j) = stats.last_color_packed;
    }
  }
}

ElevationMap fromPointCloud(const PointCloud& cloud, float resolution,
                            RasterMethod method) {
  if (cloud.empty()) return {};

  // Compute XY bounding box
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();

  for (size_t i : cloud.indices()) {
    auto pt = cloud.point(i);
    if (std::isnan(pt.x()) || std::isnan(pt.y())) continue;
    min_x = std::min(min_x, pt.x());
    min_y = std::min(min_y, pt.y());
    max_x = std::max(max_x, pt.x());
    max_y = std::max(max_y, pt.y());
  }

  // Add one cell margin so edge points are inside the map
  float width = max_x - min_x + resolution;
  float height = max_y - min_y + resolution;

  ElevationMap map;
  map.setGeometry(width, height, resolution);
  map.setPosition(
      grid_map::Position((min_x + max_x) / 2.0, (min_y + max_y) / 2.0));

  fromPointCloud(cloud, map, method);
  return map;
}

// ─── removeFloatingPoints (private) ─────────────────────────────────────

namespace {

/// Find the lowest peak in a z-histogram.
/// Returns the bin center of the lowest peak (first bin with local maximum
/// count, or the mode if no clear peak structure).
float findGroundPeak(const std::vector<float>& z_values, float bin_size) {
  if (z_values.empty()) return 0.0f;

  float z_min = *std::min_element(z_values.begin(), z_values.end());
  float z_max = *std::max_element(z_values.begin(), z_values.end());

  int n_bins = std::max(1, static_cast<int>((z_max - z_min) / bin_size) + 1);
  std::vector<int> histogram(n_bins, 0);

  for (float z : z_values) {
    int bin = std::min(static_cast<int>((z - z_min) / bin_size), n_bins - 1);
    ++histogram[bin];
  }

  // Find the bin with the most points in the lower half of the histogram.
  // This avoids selecting a high cluster (e.g., ceiling) as ground.
  int best_bin = 0;
  int best_count = 0;
  for (int i = 0; i < n_bins; ++i) {
    if (histogram[i] > best_count) {
      best_count = histogram[i];
      best_bin = i;
    }
  }

  return z_min + (best_bin + 0.5f) * bin_size;
}

/// Remove points that float above the ground peak per grid cell.
///
/// For each cell, collects all z-values, builds a histogram to find the
/// ground peak, and removes points more than height_threshold above it.
/// This removes canopy, ceilings, and SLAM artifacts while preserving
/// ground-level structures (benches, curbs, etc.).
PointCloud removeFloatingPoints(const PointCloud& cloud,
                                const ElevationMap& map,
                                float height_threshold, float bin_size) {
  // Phase 1: Collect per-cell z-values and point indices
  CellMap<std::vector<size_t>> cell_points;
  cell_points.reserve(cloud.size() / 4);

  for (size_t i : cloud.indices()) {
    auto pt = cloud.point(i);
    if (std::isnan(pt.z())) continue;

    grid_map::Index index;
    if (!map.getIndex(grid_map::Position(pt.x(), pt.y()), index)) continue;

    cell_points[index].push_back(i);
  }

  // Phase 2: Per-cell histogram → determine ground peak → mark keepers
  std::vector<bool> keep(cloud.size(), false);

  for (const auto& [index, indices] : cell_points) {
    // Collect z-values for this cell
    std::vector<float> z_values;
    z_values.reserve(indices.size());
    for (size_t i : indices) {
      z_values.push_back(cloud.point(i).z());
    }

    float ground_z = findGroundPeak(z_values, bin_size);
    float cutoff = ground_z + height_threshold;

    for (size_t i : indices) {
      if (cloud.point(i).z() <= cutoff) {
        keep[i] = true;
      }
    }
  }

  // Phase 3: Filter using nanopcl (preserves all channels)
  return nanopcl::filters::filter(cloud,
                                  [&keep](size_t i) { return keep[i]; });
}

}  // namespace

// ─── buildDEM ───────────────────────────────────────────────────────────

ElevationMap buildDEM(const PointCloud& cloud, const DEMConfig& config) {
  if (cloud.empty()) return {};

  // 1. Statistical outlier removal
  auto filtered = nanopcl::filters::statisticalOutlierRemoval(
      cloud, config.sor_k, config.sor_std_mul);

  if (filtered.empty()) return {};

  // 2. Create map geometry from bounding box (needed for histogram filter)
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();

  for (size_t i : filtered.indices()) {
    auto pt = filtered.point(i);
    if (std::isnan(pt.x()) || std::isnan(pt.y())) continue;
    min_x = std::min(min_x, pt.x());
    min_y = std::min(min_y, pt.y());
    max_x = std::max(max_x, pt.x());
    max_y = std::max(max_y, pt.y());
  }

  float width = max_x - min_x + config.resolution;
  float height = max_y - min_y + config.resolution;

  ElevationMap map;
  map.setGeometry(width, height, config.resolution);
  map.setPosition(
      grid_map::Position((min_x + max_x) / 2.0, (min_y + max_y) / 2.0));

  // 3. Per-cell histogram filter (floating point removal)
  float bin_size =
      (config.bin_size > 0.0f) ? config.bin_size : config.resolution;
  filtered = removeFloatingPoints(filtered, map, config.height_threshold,
                                  bin_size);

  // 4. Rasterization
  fromPointCloud(filtered, map, config.method);

  // 5. Inpainting
  if (config.inpaint_iterations > 0) {
    applyInpainting(map, config.inpaint_iterations, /*min_valid_neighbors=*/2,
                    /*inplace=*/true);
  }

  return map;
}

// ─── toPointCloud ───────────────────────────────────────────────────────

PointCloud toPointCloud(const ElevationMap& map) {
  const auto idx = map.indexer();
  const auto& elev_mat = map.get(layer::elevation);

  const bool has_intensity = map.exists(layer::intensity);
  const bool has_color = map.exists(layer::color);

  PointCloud cloud;
  cloud.reserve(idx.rows * idx.cols);

  for (int row = 0; row < idx.rows; ++row) {
    for (int col = 0; col < idx.cols; ++col) {
      auto [r, c] = idx(row, col);
      const float z = elev_mat(r, c);
      if (std::isnan(z)) continue;

      grid_map::Index grid_idx(r, c);
      grid_map::Position pos;
      if (!map.getPosition(grid_idx, pos)) continue;

      cloud.add(static_cast<float>(pos.x()), static_cast<float>(pos.y()), z);

      if (has_intensity) {
        float val = map.at(layer::intensity, grid_idx);
        if (!std::isnan(val)) {
          if (!cloud.hasIntensity()) cloud.useIntensity();
          cloud.intensity(cloud.size() - 1) = val;
        }
      }

      if (has_color) {
        float packed = map.at(layer::color, grid_idx);
        if (!std::isnan(packed)) {
          Eigen::Vector3i rgb;
          grid_map::colorValueToVector(packed, rgb);
          if (!cloud.hasColor()) cloud.useColor();
          cloud.color(cloud.size() - 1) = {static_cast<uint8_t>(rgb(0)),
                                            static_cast<uint8_t>(rgb(1)),
                                            static_cast<uint8_t>(rgb(2))};
        }
      }
    }
  }

  return cloud;
}

}  // namespace fastdem
