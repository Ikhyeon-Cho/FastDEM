// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * elevation_map.hpp
 *
 * 2.5D elevation map built on nanoGrid.
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
#include <nanogrid/nanogrid.hpp>
#include <initializer_list>
#include <string>
#include <unordered_map>
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

/// Cell-indexed hash map (sparse point-to-cell accumulation).
template <typename T>
using CellMap =
    std::unordered_map<nanogrid::Index, T, nanogrid::IndexHash,
                       nanogrid::IndexEqual>;

// ─── ElevationMap ───────────────────────────────────────────────────────────

/**
 * @brief 2.5D elevation map for terrain representation.
 *
 * ElevationMap extends nanogrid::GridMap with predefined layers for elevation
 * mapping: elevation, variance, count, etc. It provides
 * convenient methods for elevation access and map management.
 *
 * @note All elevation values are in meters. NaN indicates unmeasured cells.
 */
class ElevationMap : public nanogrid::GridMap {
 public:
  ElevationMap();

  ElevationMap(float width, float height, float resolution,
               const std::string& frame_id);

  void setGeometry(float width, float height, float resolution);

  bool isInitialized() const;

  bool isEmpty() const;

  bool isEmptyAt(const nanogrid::Index& index) const;

  void clearAt(const nanogrid::Index& index);

  /// Get elevation at position. Returns NaN if outside or unmeasured.
  float elevationAt(const nanogrid::Position& position) const;

  /// Get elevation at index. Returns NaN if invalid or unmeasured.
  float elevationAt(const nanogrid::Index& index) const;

  /// Check if elevation exists at position.
  bool hasElevationAt(const nanogrid::Position& position) const;

  /// Check if elevation exists at index.
  bool hasElevationAt(const nanogrid::Index& index) const;

  /// Returns a mask where finite cells are 1.0 and NaN cells are 0.0.
  Eigen::MatrixXf isFinite(const std::string& layer) const;

  /// Create a lightweight copy with only the specified layers.
  ElevationMap snapshot(std::initializer_list<std::string> layers) const;
};

inline ElevationMap::ElevationMap()
    : nanogrid::GridMap(
          {layer::elevation, layer::elevation_min, layer::elevation_max}) {}

inline ElevationMap::ElevationMap(float width, float height, float resolution,
                                  const std::string& frame_id)
    : ElevationMap() {
  setGeometry(width, height, resolution);
  setFrameId(frame_id);
}

inline void ElevationMap::setGeometry(float width, float height,
                                      float resolution) {
  nanogrid::GridMap::setGeometry(nanogrid::Length(width, height), resolution);
  clearAll();
}

inline bool ElevationMap::isInitialized() const {
  const auto& size = getSize();
  return size(0) > 0 && size(1) > 0;
}

inline bool ElevationMap::isEmpty() const {
  return get(layer::elevation).array().isNaN().all();
}

inline bool ElevationMap::isEmptyAt(const nanogrid::Index& index) const {
  return std::isnan(at(layer::elevation, index));
}

inline void ElevationMap::clearAt(const nanogrid::Index& index) {
  for (const auto& layer : getLayers()) {
    at(layer, index) = NAN;
  }
}

inline float ElevationMap::elevationAt(
    const nanogrid::Position& position) const {
  if (!isInside(position)) return NAN;
  return atPosition(layer::elevation, position);
}

inline float ElevationMap::elevationAt(const nanogrid::Index& index) const {
  return at(layer::elevation, index);
}

inline bool ElevationMap::hasElevationAt(
    const nanogrid::Position& position) const {
  return std::isfinite(elevationAt(position));
}

inline bool ElevationMap::hasElevationAt(const nanogrid::Index& index) const {
  return std::isfinite(elevationAt(index));
}

inline Eigen::MatrixXf ElevationMap::isFinite(const std::string& layer) const {
  const auto& data = get(layer).array();
  return (1.0f - (data != data).cast<float>()).matrix();
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

}  // namespace fastdem

#endif  // FASTDEM_ELEVATION_MAP_HPP
