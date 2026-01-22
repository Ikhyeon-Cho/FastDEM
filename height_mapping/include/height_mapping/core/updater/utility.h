/*
 * utility.h
 *
 * Shared utility functions for elevation updaters.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_UPDATER_UTILITY_H
#define HEIGHT_MAPPING_CORE_UPDATER_UTILITY_H

#include "height_mapping/core/layers.h"
#include "height_mapping/core/map.h"
#include "height_mapping/ops/median_filter.h"

namespace height_mapping::updater {

/**
 * @brief Create denoised elevation_max layer via median filtering.
 *
 * Copies elevation_max to elevation_max_denoised and applies median filter
 * to remove spike noise while preserving edges.
 *
 * @param map HeightMap containing elevation_max layer
 * @param kernel_size Median filter kernel size (default: 3)
 * @param min_valid_neighbors Minimum valid neighbors for filtering (default: 5)
 */
inline void denoiseElevationMax(HeightMap& map, int kernel_size = 3,
                                int min_valid_neighbors = 5) {
  if (!map.exists(layer::elevation_max)) {
    return;
  }

  // 1. Ensure denoised layer exists
  if (!map.exists(layer::elevation_max_denoised)) {
    map.add(layer::elevation_max_denoised, NAN);
  }

  // 2. Copy elevation_max -> elevation_max_denoised
  map.get(layer::elevation_max_denoised) = map.get(layer::elevation_max);

  // 3. Apply median filter to denoised layer (in-place)
  config::MedianFilter cfg;
  cfg.enabled = true;
  cfg.kernel_size = kernel_size;
  cfg.min_valid_neighbors = min_valid_neighbors;
  cfg.target_layers = {layer::elevation_max_denoised};

  ops::applyMedianFilter(map, cfg);
}

}  // namespace height_mapping::updater

#endif  // HEIGHT_MAPPING_CORE_UPDATER_UTILITY_H
