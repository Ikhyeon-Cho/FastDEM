// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * elevation_map.hpp
 *
 * 2.5D elevation map built on grid_map.
 * Includes layer name constants.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_ELEVATION_MAP_HPP
#define FASTDEM_ELEVATION_MAP_HPP

#include <cmath>
#include <grid_map_core/grid_map_core.hpp>
#include <initializer_list>
#include <string>
#include <vector>

namespace fastdem {

namespace layer {
constexpr auto elevation = "elevation";
constexpr auto elevation_min = "elevation_min";
constexpr auto elevation_max = "elevation_max";
constexpr auto variance = "variance";
constexpr auto n_points = "n_points";
constexpr auto upper_bound = "upper_bound";
constexpr auto lower_bound = "lower_bound";

// Per-frame layers
constexpr auto obstacle = "obstacle";
constexpr auto intensity = "intensity";
constexpr auto color = "color";

/// Internal layers use '_' prefix and are excluded from visualization.
inline bool isInternal(const std::string& name) {
  return !name.empty() && name[0] == '_';
}
}  // namespace layer

// ─── MapIndexer ─────────────────────────────────────────────────────────────

class ElevationMap;  // forward declaration

/**
 * @brief Lightweight helper for iterating over ElevationMap grid cells.
 *
 * Handles the circular buffer index mapping internally so callers can iterate
 * with plain (row, col) loops and access Eigen matrices without knowing about
 * the underlying buffer layout.
 *
 */
struct MapIndexer {
  const int rows, cols;
  const float resolution;

  explicit MapIndexer(const ElevationMap& map);

  /// Grid (row, col) → matrix position for Eigen access.
  std::pair<int, int> operator()(int row, int col) const {
    int r = row + sr_;
    if (r >= rows) r -= rows;
    int c = col + sc_;
    if (c >= cols) c -= cols;
    return {r, c};
  }

  /// Check if (row, col) is within [0, rows) × [0, cols).
  bool contains(int row, int col) const {
    return row >= 0 && row < rows && col >= 0 && col < cols;
  }

  /// Relative cell offset within a circular radius.
  struct Neighbor {
    int dr, dc;
    float dist_sq;  ///< Squared distance in meters.
  };

  /// Precompute neighbor offsets within a circular radius.
  /// @note Includes the center cell (dr=0, dc=0, dist_sq=0).
  std::vector<Neighbor> circleNeighbors(float radius) const {
    std::vector<Neighbor> offsets;
    const int r_cells = static_cast<int>(std::ceil(radius / resolution));
    const float radius_sq = radius * radius;
    for (int dr = -r_cells; dr <= r_cells; ++dr) {
      for (int dc = -r_cells; dc <= r_cells; ++dc) {
        const float dsq =
            resolution * resolution * static_cast<float>(dr * dr + dc * dc);
        if (dsq <= radius_sq) offsets.push_back({dr, dc, dsq});
      }
    }
    return offsets;
  }

 private:
  int sr_, sc_;
};

// ─── ElevationMap ───────────────────────────────────────────────────────────

/**
 * @brief 2.5D elevation map for terrain representation.
 *
 * ElevationMap extends grid_map::GridMap with predefined layers for elevation
 * mapping: elevation, variance, count, etc. It provides
 * convenient methods for elevation access and map management.
 *
 * @note All elevation values are in meters. NaN indicates unmeasured cells.
 */
class ElevationMap : public grid_map::GridMap {
 public:
  ElevationMap();

  ElevationMap(float width, float height, float resolution,
               const std::string& frame_id);

  void setGeometry(float width, float height, float resolution);

  bool isInitialized() const;

  bool isEmpty() const;

  bool isEmptyAt(const grid_map::Index& index) const;

  void clearAt(const grid_map::Index& index);

  /// Get elevation at position. Returns NaN if outside or unmeasured.
  float elevationAt(const grid_map::Position& position) const;

  /// Get elevation at index. Returns NaN if invalid or unmeasured.
  float elevationAt(const grid_map::Index& index) const;

  /// Check if elevation exists at position.
  bool hasElevationAt(const grid_map::Position& position) const;

  /// Check if elevation exists at index.
  bool hasElevationAt(const grid_map::Index& index) const;

  /// Create a MapIndexer for efficient grid iteration.
  MapIndexer indexer() const;

  /// Create a lightweight copy with only the specified layers.
  ElevationMap snapshot(std::initializer_list<std::string> layers) const;
};

inline ElevationMap::ElevationMap()
    : grid_map::GridMap(
          {layer::elevation, layer::elevation_min, layer::elevation_max}) {}

inline ElevationMap::ElevationMap(float width, float height, float resolution,
                                  const std::string& frame_id)
    : ElevationMap() {
  setGeometry(width, height, resolution);
  setFrameId(frame_id);
}

inline void ElevationMap::setGeometry(float width, float height,
                                      float resolution) {
  grid_map::GridMap::setGeometry(grid_map::Length(width, height), resolution);
  clearAll();
}

inline bool ElevationMap::isInitialized() const {
  const auto& size = getSize();
  return size(0) > 0 && size(1) > 0;
}

inline bool ElevationMap::isEmpty() const {
  return get(layer::elevation).array().isNaN().all();
}

inline bool ElevationMap::isEmptyAt(const grid_map::Index& index) const {
  return std::isnan(at(layer::elevation, index));
}

inline void ElevationMap::clearAt(const grid_map::Index& index) {
  for (const auto& layer : getLayers()) {
    at(layer, index) = NAN;
  }
}

inline float ElevationMap::elevationAt(
    const grid_map::Position& position) const {
  if (!isInside(position)) return NAN;
  return atPosition(layer::elevation, position);
}

inline float ElevationMap::elevationAt(const grid_map::Index& index) const {
  return at(layer::elevation, index);
}

inline bool ElevationMap::hasElevationAt(
    const grid_map::Position& position) const {
  return std::isfinite(elevationAt(position));
}

inline bool ElevationMap::hasElevationAt(const grid_map::Index& index) const {
  return std::isfinite(elevationAt(index));
}

inline ElevationMap ElevationMap::snapshot(
    std::initializer_list<std::string> layers) const {
  ElevationMap snap;
  snap.setGeometry(getLength()(0), getLength()(1), getResolution());
  snap.setFrameId(getFrameId());
  snap.setPosition(getPosition());
  snap.setStartIndex(getStartIndex());
  snap.setTimestamp(getTimestamp());
  for (const auto& name : layers) {
    if (!exists(name)) continue;
    if (snap.exists(name))
      snap.get(name) = get(name);
    else
      snap.add(name, get(name));
  }
  return snap;
}

// ─── MapIndexer inline definitions ──────────────────────────────────────────

inline MapIndexer::MapIndexer(const ElevationMap& map)
    : rows(map.getSize()(0)),
      cols(map.getSize()(1)),
      resolution(map.getResolution()),
      sr_(map.getStartIndex()(0)),
      sc_(map.getStartIndex()(1)) {}

inline MapIndexer ElevationMap::indexer() const { return MapIndexer(*this); }

}  // namespace fastdem

#endif  // FASTDEM_ELEVATION_MAP_HPP
