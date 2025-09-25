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

namespace height_mapping {

namespace layer {

// Core layers (always initialized)
static constexpr const char *elevation = "elevation";
static constexpr const char *variance = "variance";
static constexpr const char *elevation_min = "elevation_min";
static constexpr const char *elevation_max = "elevation_max";
static constexpr const char *count = "measurement_count";

// Statistical quality layers
static constexpr const char *standard_error = "standard_error";
static constexpr const char *confidence_interval = "confidence_interval_95";

// Optional layers
static constexpr const char *confidence = "confidence";
static constexpr const char *traversability = "traversability";
static constexpr const char *raycasting = "raycasting";
static constexpr const char *scan_elevation = "scan_elevation";

// Sensor data layers
static constexpr const char *intensity = "intensity";
static constexpr const char *color_r = "color_r";
static constexpr const char *color_g = "color_g";
static constexpr const char *color_b = "color_b";
} // namespace layer

class HeightMap : public grid_map::GridMap {
public:
  using Ptr = std::shared_ptr<HeightMap>;
  using ConstPtr = std::shared_ptr<const HeightMap>;
  /**
   * @brief Default constructor - initializes core layers
   */
  HeightMap()
      : grid_map::GridMap({layer::elevation, layer::variance,
                           layer::elevation_min, layer::elevation_max,
                           layer::count}) {
    // Initialize layers with appropriate defaults
    get(layer::count).setConstant(0.0f);
    setBasicLayers({layer::elevation});
  }

  /**
   * @brief Initialize map geometry
   */
  void initialize(float width, float height, float resolution) {
    setGeometry(grid_map::Length(width, height), resolution);
    clearAll();

    // Reset statistics to defaults
    get(layer::variance).setConstant(1000.0f);
    get(layer::count).setConstant(0.0f);
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
    if (!isInside(pos))
      return false;

    atPosition(layer::elevation, pos) = elevation;
    return true;
  }

  /**
   * @brief Set elevation with variance at position
   */
  bool setElevation(float x, float y, float elevation, float variance) {
    grid_map::Position pos(x, y);
    if (!isInside(pos))
      return false;

    atPosition(layer::elevation, pos) = elevation;
    atPosition(layer::variance, pos) = variance;
    return true;
  }

  /**
   * @brief Get elevation at position
   */
  bool getElevation(float x, float y, float &elevation) const {
    grid_map::Position pos(x, y);
    if (!isInside(pos))
      return false;

    elevation = atPosition(layer::elevation, pos);
    return std::isfinite(elevation);
  }

  /**
   * @brief Check if a cell is empty (no measurements)
   */
  bool isEmptyAt(const grid_map::Index &index) const {
    return !isValid(index) || std::isnan(at(layer::elevation, index));
  }

  /**
   * @brief Get or create standard error layer
   */
  Eigen::MatrixXf &getStandardError() {
    if (!exists(layer::standard_error)) {
      add(layer::standard_error);
      get(layer::standard_error).setConstant(NAN);
    }
    return get(layer::standard_error);
  }

  const Eigen::MatrixXf &getStandardError() const {
    return get(layer::standard_error);
  }

  /**
   * @brief Get or create confidence interval layer (95%)
   */
  Eigen::MatrixXf &getConfidenceInterval() {
    if (!exists(layer::confidence_interval)) {
      add(layer::confidence_interval);
      get(layer::confidence_interval).setConstant(NAN);
    }
    return get(layer::confidence_interval);
  }

  const Eigen::MatrixXf &getConfidenceInterval() const {
    return get(layer::confidence_interval);
  }

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

} // namespace height_mapping

#endif // HEIGHT_MAPPING_CORE_DATA_HEIGHT_MAP_H