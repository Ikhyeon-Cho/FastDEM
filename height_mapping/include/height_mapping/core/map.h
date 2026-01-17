/*
 * map.h
 *
 * 2.5D elevation map built on grid_map.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_MAP_H
#define HEIGHT_MAPPING_CORE_MAP_H

#include <cmath>
#include <grid_map_core/grid_map_core.hpp>

#include "height_mapping/core/layers.h"

namespace height_mapping {

/**
 * @brief 2.5D elevation map for terrain representation.
 *
 * HeightMap extends grid_map::GridMap with predefined layers for elevation
 * mapping: elevation, variance, count, persistence, etc. It provides
 * convenient methods for elevation access and map management.
 *
 * @note All elevation values are in meters. NaN indicates unmeasured cells.
 *
 * Example usage:
 * @code
 *   HeightMap map({20.0f, 20.0f, 0.1f, "map"});
 *   map.setElevation(1.0f, 2.0f, 0.5f);
 *   float z;
 *   if (map.getElevation(1.0f, 2.0f, z)) {
 *     std::cout << "Elevation: " << z << std::endl;
 *   }
 * @endcode
 */
class HeightMap : public grid_map::GridMap {
 public:
  /// Initial variance for unmeasured cells (high uncertainty)
  static constexpr float initial_variance = 1000.0f;

  /**
   * @brief Configuration for HeightMap initialization.
   */
  struct Config {
    float width = 10.0f;          ///< Map width in meters
    float height = 10.0f;         ///< Map height in meters
    float resolution = 0.1f;      ///< Cell size in meters
    std::string frame_id{"map"};  ///< Coordinate frame ID
  };

  /**
   * @brief Default constructor.
   *
   * Creates a HeightMap with predefined layers but no geometry.
   * Call setGeometry() before use.
   */
  HeightMap();

  /**
   * @brief Construct with configuration.
   *
   * @param config Map configuration (size, resolution, frame)
   */
  explicit HeightMap(const Config& config);

  /**
   * @brief Initialize map geometry.
   *
   * Sets the map size and resolution, and clears all data.
   * This resets variance to initial_variance and count/persistence to 0.
   *
   * @param width Map width in meters
   * @param height Map height in meters
   * @param resolution Cell size in meters
   */
  void setGeometry(float width, float height, float resolution);

  /**
   * @brief Check if map geometry has been initialized.
   *
   * @return true if map has valid size (rows > 0 and cols > 0)
   */
  bool isInitialized() const;

  /**
   * @brief Check if map is empty (all cells unmeasured).
   *
   * @return true if all elevation values are NaN
   */
  bool isEmpty() const;

  /**
   * @brief Clear all elevation data while keeping geometry.
   *
   * Resets elevation to NaN, variance to initial_variance,
   * and count/persistence to 0.
   */
  void clear();

  /**
   * @brief Set elevation at a world position.
   *
   * @param x X coordinate in meters (map frame)
   * @param y Y coordinate in meters (map frame)
   * @param elevation Elevation value in meters
   * @return true if position is inside map bounds
   */
  bool setElevation(float x, float y, float elevation);

  /**
   * @brief Set elevation with variance at a world position.
   *
   * @param x X coordinate in meters (map frame)
   * @param y Y coordinate in meters (map frame)
   * @param elevation Elevation value in meters
   * @param variance Elevation variance (uncertainty)
   * @return true if position is inside map bounds
   */
  bool setElevation(float x, float y, float elevation, float variance);

  /**
   * @brief Get elevation at a world position.
   *
   * @param x X coordinate in meters (map frame)
   * @param y Y coordinate in meters (map frame)
   * @param[out] elevation Retrieved elevation value
   * @return true if position is inside map and has valid (finite) elevation
   */
  bool getElevation(float x, float y, float& elevation) const;

  /**
   * @brief Check if a cell is empty (no measurements).
   *
   * @param index Grid cell index
   * @return true if cell is invalid or elevation is NaN
   */
  bool isEmptyAt(const grid_map::Index& index) const;
};

// =============================================================================
// Inline Implementation
// =============================================================================

inline HeightMap::HeightMap()
    : grid_map::GridMap({layer::elevation, layer::variance,
                         layer::elevation_min, layer::elevation_max,
                         layer::count, layer::persistence}) {
  get(layer::count).setConstant(0.0f);
  get(layer::persistence).setConstant(0.0f);
}

inline HeightMap::HeightMap(const Config& config) : HeightMap() {
  setGeometry(config.width, config.height, config.resolution);
  setFrameId(config.frame_id);
}

inline void HeightMap::setGeometry(float width, float height,
                                   float resolution) {
  grid_map::GridMap::setGeometry(grid_map::Length(width, height), resolution);
  clearAll();

  // Reset statistics to defaults
  get(layer::variance).setConstant(initial_variance);
  get(layer::count).setConstant(0.0f);
  get(layer::persistence).setConstant(0.0f);
}

inline bool HeightMap::isInitialized() const {
  const auto& size = getSize();
  return size(0) > 0 && size(1) > 0;
}

inline bool HeightMap::isEmpty() const {
  return get(layer::elevation).array().isNaN().all();
}

inline void HeightMap::clear() {
  clearAll();
  get(layer::variance).setConstant(initial_variance);
  get(layer::count).setConstant(0.0f);
  get(layer::persistence).setConstant(0.0f);
}

inline bool HeightMap::setElevation(float x, float y, float elevation) {
  grid_map::Position pos(x, y);
  if (!isInside(pos)) return false;

  atPosition(layer::elevation, pos) = elevation;
  return true;
}

inline bool HeightMap::setElevation(float x, float y, float elevation,
                                    float variance) {
  grid_map::Position pos(x, y);
  if (!isInside(pos)) return false;

  atPosition(layer::elevation, pos) = elevation;
  atPosition(layer::variance, pos) = variance;
  return true;
}

inline bool HeightMap::getElevation(float x, float y, float& elevation) const {
  grid_map::Position pos(x, y);
  if (!isInside(pos)) return false;

  elevation = atPosition(layer::elevation, pos);
  return std::isfinite(elevation);
}

inline bool HeightMap::isEmptyAt(const grid_map::Index& index) const {
  return !isValid(index) || std::isnan(at(layer::elevation, index));
}

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_CORE_MAP_H
