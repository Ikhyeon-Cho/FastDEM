/*
 * height_map.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_HEIGHT_MAP_H
#define HEIGHT_MAPPING_HEIGHT_MAP_H

#include <grid_map_core/grid_map_core.hpp>

namespace height_mapping {
namespace layer {
// Core layers (initialized in constructor)
constexpr const char* elevation = "elevation";
constexpr const char* variance = "variance";
constexpr const char* elevation_min = "elevation_min";
constexpr const char* elevation_max = "elevation_max";
constexpr const char* count = "measurement_count";

// Derived statistics (computed in updateHeightMap)
constexpr const char* standard_error = "standard_error";
constexpr const char* confidence_interval = "confidence_interval_95";

// Extension layers (add at use site when needed)
constexpr const char* intensity = "intensity";
constexpr const char* r = "r";
constexpr const char* g = "g";
constexpr const char* b = "b";
constexpr const char* label = "label";
constexpr const char* ray_height = "ray_height";
constexpr const char* traversability = "traversability";
}  // namespace layer

class HeightMap : public grid_map::GridMap {
 public:
  struct Config {
    float width = 10.0f;
    float height = 10.0f;
    float resolution = 0.1f;
    std::string frame_id = "map";
  };

  HeightMap()
      : grid_map::GridMap({layer::elevation, layer::variance,
                           layer::elevation_min, layer::elevation_max,
                           layer::count}) {
    get(layer::count).setConstant(0.0f);
    setBasicLayers({layer::elevation});
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
    get(layer::variance).setConstant(1000.0f);
    get(layer::count).setConstant(0.0f);
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
  bool empty() const { return get(layer::elevation).array().isNaN().all(); }

  /**
   * @brief Clear all elevation data (keeps geometry)
   */
  void clear() {
    clearAll();
    get(layer::variance).setConstant(1000.0f);
    get(layer::count).setConstant(0.0f);
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

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_HEIGHT_MAP_H