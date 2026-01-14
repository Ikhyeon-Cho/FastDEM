/*
 * height_map.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_HEIGHT_MAP_H
#define HEIGHT_MAP_HEIGHT_MAP_H

#include <grid_map_core/grid_map_core.hpp>

#include "height_map/layers.h"

namespace height_map {

class HeightMap : public grid_map::GridMap {
 public:
  static constexpr float initial_variance = 1000.0f;

  struct Config {
    float width = 10.0f;
    float height = 10.0f;
    float resolution = 0.1f;
    std::string frame_id{"map"};
  };

  HeightMap()
      : grid_map::GridMap({layer::elevation, layer::variance,
                           layer::elevation_min, layer::elevation_max,
                           layer::count, layer::persistence}) {
    get(layer::count).setConstant(0.0f);
    get(layer::persistence).setConstant(0.0f);
  }

  explicit HeightMap(const Config& config) : HeightMap() {
    setGeometry(config.width, config.height, config.resolution);
    setFrameId(config.frame_id);
  }

  /**
   * @brief Initialize map geometry
   */
  void setGeometry(float width, float height, float resolution) {
    grid_map::GridMap::setGeometry(grid_map::Length(width, height), resolution);
    clearAll();

    // Reset statistics to defaults
    get(layer::variance).setConstant(initial_variance);
    get(layer::count).setConstant(0.0f);
    get(layer::persistence).setConstant(0.0f);
  }

  /**
   * @brief Check if map has been initialized
   */
  bool isInitialized() const {
    const auto& size = getSize();
    return size(0) > 0 && size(1) > 0;
  }

  /**
   * @brief Check if map is empty (all cells unset)
   */
  bool isEmpty() const { return get(layer::elevation).array().isNaN().all(); }

  /**
   * @brief Clear all elevation data (keeps geometry)
   */
  void clear() {
    clearAll();
    get(layer::variance).setConstant(initial_variance);
    get(layer::count).setConstant(0.0f);
    get(layer::persistence).setConstant(0.0f);
  }

  /**
   * @brief Set elevation at position
   */
  bool setElevation(float x, float y, float elevation) {
    grid_map::Position pos(x, y);
    if (!isInside(pos)) return false;

    atPosition(layer::elevation, pos) = elevation;
    return true;
  }

  /**
   * @brief Set elevation with variance at position
   */
  bool setElevation(float x, float y, float elevation, float variance) {
    grid_map::Position pos(x, y);
    if (!isInside(pos)) return false;

    atPosition(layer::elevation, pos) = elevation;
    atPosition(layer::variance, pos) = variance;
    return true;
  }

  /**
   * @brief Get elevation at position
   */
  bool getElevation(float x, float y, float& elevation) const {
    grid_map::Position pos(x, y);
    if (!isInside(pos)) return false;

    elevation = atPosition(layer::elevation, pos);
    return std::isfinite(elevation);
  }

  /**
   * @brief Check if a cell is empty (no measurements)
   */
  bool isEmptyAt(const grid_map::Index& index) const {
    return !isValid(index) || std::isnan(at(layer::elevation, index));
  }
};

}  // namespace height_map

#endif  // HEIGHT_MAP_HEIGHT_MAP_H
