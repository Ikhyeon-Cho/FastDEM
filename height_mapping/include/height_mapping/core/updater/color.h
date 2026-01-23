/*
 * color.h
 *
 * RGB color layer updater.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_UPDATER_COLOR_H
#define HEIGHT_MAPPING_CORE_UPDATER_COLOR_H

#include <cmath>
#include <limits>

#include <grid_map_core/GridMapMath.hpp>
#include <nanopcl/core/point.hpp>

#include "height_mapping/core/layers.h"
#include "height_mapping/core/map.h"

namespace height_mapping::updater {

/**
 * @brief RGB color layer updater.
 *
 * Stores RGB color as packed float for grid_map_rviz_plugin compatibility.
 * Uses simple overwrite strategy since gridMaxZ filter ensures single point
 * per cell.
 *
 * Layers created:
 * - color: Packed RGBA as float (grid_map standard format)
 */
class Color {
 public:
  Color() = default;

  /**
   * @brief Initialize layers on the height map.
   *
   * Creates color layer and caches matrix pointer.
   */
  void initialize(HeightMap& map) {
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    if (!map.exists(layer::color)) map.add(layer::color, kNaN);
    color_mat_ = &map.get(layer::color);
  }

  /**
   * @brief Update color at a cell.
   *
   * Packs RGB into float using grid_map format for RViz visualization.
   *
   * @param index Pre-computed cell index
   * @param color RGB color value
   */
  void update(const grid_map::Index& index, const npcl::Color& color) {
    const int i = index(0);
    const int j = index(1);

    // Pack RGB to float using grid_map utility
    Eigen::Vector3i rgb(color.r(), color.g(), color.b());
    float packed;
    grid_map::colorVectorToValue(rgb, packed);
    (*color_mat_)(i, j) = packed;
  }

  /**
   * @brief Finalize (no-op for color).
   */
  void finalize() {}

  std::string name() const { return "Color"; }

 private:
  grid_map::Matrix* color_mat_ = nullptr;
};

}  // namespace height_mapping::updater

#endif  // HEIGHT_MAPPING_CORE_UPDATER_COLOR_H
