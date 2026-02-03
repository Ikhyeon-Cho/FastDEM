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

namespace fastdem {

// ─── Layer name constants ───────────────────────────────────────────────────

namespace layer {

// Core layers
constexpr auto elevation = "elevation";
constexpr auto elevation_min = "elevation_min";
constexpr auto elevation_max = "elevation_max";
constexpr auto elevation_max_smoothed = "elevation_max_smoothed";

// Layers for Kalman estimator
constexpr auto state = "state";  // x (Kalman state estimate)
constexpr auto variance = "variance";
constexpr auto upper_bound = "upper_bound";
constexpr auto lower_bound = "lower_bound";
constexpr auto uncertainty_range = "uncertainty_range";  // upper - lower

// Layers for Welford's algorithm
constexpr auto mean = "mean";
constexpr auto sample_count = "sample_count";
constexpr auto sample_variance = "sample_variance";
constexpr auto standard_error = "standard_error";
constexpr auto conf_interval_95 = "conf_interval_95";

// Layers for P2 quantile estimator (marker heights)
constexpr auto p2_q0 = "p2_q0";  // 1st percentile (lower bound)
constexpr auto p2_q1 = "p2_q1";  // 16th percentile
constexpr auto p2_q2 = "p2_q2";  // 50th percentile (median)
constexpr auto p2_q3 = "p2_q3";  // 84th percentile
constexpr auto p2_q4 = "p2_q4";  // 99th percentile (upper bound)
// Layers for P2 quantile estimator (marker positions)
constexpr auto p2_n0 = "p2_n0";
constexpr auto p2_n1 = "p2_n1";
constexpr auto p2_n2 = "p2_n2";
constexpr auto p2_n3 = "p2_n3";
constexpr auto p2_n4 = "p2_n4";

// Derived statistics
constexpr auto elevation_inpainted = "elevation_inpainted";
constexpr auto conflict_count = "conflict_count";  // Raycasting temporal voting

// Extension layers (add at use site when needed)
constexpr auto intensity = "intensity";
constexpr auto color = "color";  // Packed RGBA for grid_map_rviz_plugin
constexpr auto label = "label";
constexpr auto raycasting_upper_bound = "raycasting_upper_bound";
constexpr auto traversability = "traversability";

}  // namespace layer

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
};

inline ElevationMap::ElevationMap() : grid_map::GridMap({layer::elevation}) {
  setBasicLayers({layer::elevation});
}

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
  return !isValid(index) || std::isnan(at(layer::elevation, index));
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
  if (!isValid(index)) return NAN;
  return at(layer::elevation, index);
}

inline bool ElevationMap::hasElevationAt(
    const grid_map::Position& position) const {
  return std::isfinite(elevationAt(position));
}

inline bool ElevationMap::hasElevationAt(const grid_map::Index& index) const {
  return std::isfinite(elevationAt(index));
}

}  // namespace fastdem

#endif  // FASTDEM_ELEVATION_MAP_HPP
