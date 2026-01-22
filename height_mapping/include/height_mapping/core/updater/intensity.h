/*
 * intensity.h
 *
 * Intensity layer updater using max operation.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_UPDATER_INTENSITY_H
#define HEIGHT_MAPPING_CORE_UPDATER_INTENSITY_H

#include <cmath>

#include "height_mapping/core/layers.h"
#include "height_mapping/core/map.h"

namespace height_mapping::updater {

/**
 * @brief Intensity layer updater using max operation.
 *
 * Stores the maximum intensity value observed at each cell.
 * Simple and robust for terrain visualization.
 *
 * Layers created:
 * - intensity: Maximum intensity value per cell
 */
class Intensity {
 public:
  Intensity() = default;

  /**
   * @brief Initialize layers on the height map.
   *
   * Creates intensity layer and caches matrix pointer.
   */
  void initialize(HeightMap& map) {
    if (!map.exists(layer::intensity)) map.add(layer::intensity, NAN);
    intensity_mat_ = &map.get(layer::intensity);
  }

  /**
   * @brief Update intensity at a cell using max operation.
   *
   * @param index Pre-computed cell index
   * @param intensity Intensity value
   */
  void update(const grid_map::Index& index, float intensity) {
    const int i = index(0);
    const int j = index(1);
    float& stored = (*intensity_mat_)(i, j);

    if (std::isnan(stored) || intensity > stored) {
      stored = intensity;
    }
  }

  /**
   * @brief Finalize (no-op for max intensity).
   */
  void finalize() {}

  std::string name() const { return "Intensity"; }

 private:
  grid_map::Matrix* intensity_mat_ = nullptr;
};

}  // namespace height_mapping::updater

#endif  // HEIGHT_MAPPING_CORE_UPDATER_INTENSITY_H
