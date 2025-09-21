/*
 * height_map.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_DATA_HEIGHT_MAP_H
#define HEIGHT_MAPPING_CORE_DATA_HEIGHT_MAP_H

#include <grid_map_core/grid_map_core.hpp>

namespace height_map {

class HeightMap : public grid_map::GridMap {
public:
  // ============== LAYER DEFINITIONS ==============

  // Core layers (initialized by default)
  static constexpr const char *ELEVATION = "elevation";
  static constexpr const char *VARIANCE = "variance";

  // Statistics layers (optional)
  static constexpr const char *MIN_HEIGHT = "min_height";
  static constexpr const char *MAX_HEIGHT = "max_height";
  static constexpr const char *COUNT = "measurement_count";
  static constexpr const char *CONFIDENCE = "confidence";

  // Processing layers (optional)
  static constexpr const char *RAYCASTING = "raycasting";
  static constexpr const char *SCAN_HEIGHT = "scan_height";
  static constexpr const char *TRAVERSABILITY = "traversability";

  // Sensor data layers (optional)
  static constexpr const char *INTENSITY = "intensity";
  static constexpr const char *COLOR_R = "color_r";
  static constexpr const char *COLOR_G = "color_g";
  static constexpr const char *COLOR_B = "color_b";

  /**
   * @brief Default constructor - initializes elevation and variance layers
   */
  HeightMap() : grid_map::GridMap({ELEVATION, VARIANCE}) {
    get(VARIANCE).setConstant(1000.0f);
    setBasicLayers({ELEVATION});
  }

  /**
   * @brief Initialize map geometry
   */
  void initialize(float width, float height, float resolution) {
    setGeometry(grid_map::Length(width, height), resolution);
    clearAll();
    get(VARIANCE).setConstant(1000.0f);
  }

  /**
   * @brief Check if map has been initialized
   */
  bool isInitialized() const {
    const auto &size = getSize();
    return size(0) > 0 && size(1) > 0;
  }

  /**
   * @brief Check if map is empty (all cells unset)
   */
  bool empty() const { return get(ELEVATION).array().isNaN().all(); }

  /**
   * @brief Clear all height data (keeps geometry)
   */
  void clear() {
    clearAll();
    get(VARIANCE).setConstant(1000.0f);
  }

  /**
   * @brief Set height at position
   */
  bool setHeight(float x, float y, float height) {
    grid_map::Position pos(x, y);
    if (!isInside(pos))
      return false;

    atPosition(ELEVATION, pos) = height;
    return true;
  }

  /**
   * @brief Set height with variance at position
   */
  bool setHeight(float x, float y, float height, float variance) {
    grid_map::Position pos(x, y);
    if (!isInside(pos))
      return false;

    atPosition(ELEVATION, pos) = height;
    atPosition(VARIANCE, pos) = variance;
    return true;
  }

  /**
   * @brief Get height at position
   */
  bool getHeight(float x, float y, float &height) const {
    grid_map::Position pos(x, y);
    if (!isInside(pos))
      return false;

    height = atPosition(ELEVATION, pos);
    return std::isfinite(height);
  }

  /**
   * @brief Direct access to elevation matrix
   */
  Eigen::MatrixXf &getElevation() { return get(ELEVATION); }

  const Eigen::MatrixXf &getElevation() const { return get(ELEVATION); }

  /**
   * @brief Direct access to variance matrix
   */
  Eigen::MatrixXf &getVariance() { return get(VARIANCE); }

  const Eigen::MatrixXf &getVariance() const { return get(VARIANCE); }

  /**
   * @brief Get underlying GridMap (for compatibility)
   */
  const grid_map::GridMap &getGridMap() const { return *this; }
  grid_map::GridMap &getGridMap() { return *this; }

  // Inherit useful methods from GridMap:
  // - getResolution()
  // - isInside(position)
  // - getSize()
  // - move(position)
  // - getPosition()
  // - setFrameId(frame_id)
  // - getFrameId()
  // - atPosition(layer, position)
  // - add(layer)
  // - exists(layer)
  // - get(layer)
  // And many more...
};

} // namespace height_map

#endif // HEIGHT_MAPPING_CORE_DATA_HEIGHT_MAP_H