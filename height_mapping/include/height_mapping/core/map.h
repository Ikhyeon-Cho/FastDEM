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
#include <memory>

#include "height_mapping/core/layers.h"

namespace height_mapping {

/**
 * @brief 2.5D elevation map for terrain representation.
 *
 * HeightMap extends grid_map::GridMap with predefined layers for elevation
 * mapping: elevation, variance, count, etc. It provides
 * convenient methods for elevation access and map management.
 *
 * @note All elevation values are in meters. NaN indicates unmeasured cells.
 *
 * Example usage:
 * @code
 *   HeightMap map(20.0f, 20.0f, 0.1f, "map");
 *   map.setElevation(1.0f, 2.0f, 0.5f);
 *   float z;
 *   if (map.getElevation(1.0f, 2.0f, z)) {
 *     std::cout << "Elevation: " << z << std::endl;
 *   }
 * @endcode
 */
class HeightMap : public grid_map::GridMap {
 public:
  /**
   * @brief Default constructor.
   *
   * Creates a HeightMap with predefined layers but no geometry.
   * Call setGeometry() before use.
   */
  HeightMap();

  /**
   * @brief Construct with geometry parameters.
   *
   * @param width Map width in meters
   * @param height Map height in meters
   * @param resolution Cell size in meters
   * @param frame_id Coordinate frame ID
   */
  HeightMap(float width, float height, float resolution,
            const std::string& frame_id);

  /**
   * @brief Initialize map geometry.
   *
   * Sets the map size and resolution, and clears all data.
   * All layers are reset to NaN.
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

  // Inherited from GridMap:
  // - clear(const std::string& layer)  // Clear specific layer
  // - clearAll()                        // Clear all layers
  // - clearBasic()                      // Clear basic layers

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

  /**
   * @brief Clear a single cell's data (all layers to NaN).
   *
   * Sets all layers at the given cell to NaN. Updaters will re-initialize
   * the cell on the next measurement (they check for NaN in update()).
   *
   * @param index Grid cell index
   */
  void clearAt(const grid_map::Index& index);
};

// =============================================================================
// Inline Implementation
// =============================================================================

inline HeightMap::HeightMap() : grid_map::GridMap({layer::elevation}) {}

inline HeightMap::HeightMap(float width, float height, float resolution,
                            const std::string& frame_id)
    : HeightMap() {
  setGeometry(width, height, resolution);
  setFrameId(frame_id);
}

inline void HeightMap::setGeometry(float width, float height,
                                   float resolution) {
  grid_map::GridMap::setGeometry(grid_map::Length(width, height), resolution);
  clearAll();  // Sets all layers to NaN
}

inline bool HeightMap::isInitialized() const {
  const auto& size = getSize();
  return size(0) > 0 && size(1) > 0;
}

inline bool HeightMap::isEmpty() const {
  return get(layer::elevation).array().isNaN().all();
}

inline bool HeightMap::setElevation(float x, float y, float elevation) {
  grid_map::Position pos(x, y);
  if (!isInside(pos)) return false;

  atPosition(layer::elevation, pos) = elevation;
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

inline void HeightMap::clearAt(const grid_map::Index& index) {
  for (const auto& layer : getLayers()) {
    at(layer, index) = NAN;
  }
}

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_CORE_MAP_H
