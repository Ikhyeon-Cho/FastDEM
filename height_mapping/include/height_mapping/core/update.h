/*
 * update.h
 *
 * Fused multi-layer map update using variadic templates.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_UPDATE_H
#define HEIGHT_MAPPING_CORE_UPDATE_H

#include <nanopcl/core.hpp>

#include "height_mapping/core/map.h"

namespace height_mapping {

/**
 * @brief Fused multi-layer map update function.
 *
 * Updates multiple layers in a single pass over the point cloud.
 * Key optimization: index calculation happens once per point, then
 * all layer updaters receive the pre-computed index.
 *
 * Each layer class must implement:
 *   - void initialize(HeightMap& map)
 *   - void update(HeightMap& map, const grid_map::Index& index, const Point&
 * point)
 *   - void finalize(HeightMap& map)
 *
 * Benefits:
 * - Single loop over the point cloud (N iterations)
 * - Single index calculation per point (N calculations total)
 * - Cache-friendly memory access pattern
 *
 * Without this optimization, K layers would require:
 * - K loops OR N*K index calculations
 *
 * @tparam Layers Layer classes with initialize/update/finalize interface
 * @param map Height map to update
 * @param cloud Point cloud in map frame
 * @param updaters Layer updater instances
 *
 * Example usage:
 * @code
 *   updater::Elevation elevation(estimator);
 *   updater::Intensity intensity;
 *
 *   height_mapping::update(map, cloud, elevation, intensity);
 * @endcode
 */
template <typename... Layers>
void update(HeightMap& map, const npcl::PointCloud& cloud,
            Layers&... updaters) {
  if (cloud.empty()) {
    return;
  }

  // Phase 1: Initialize all updaters (create missing layers, cache matrix refs)
  (updaters.initialize(map), ...);

  // Phase 2: Single loop - compute index once, update all layers
  for (const auto& point : cloud) {
    grid_map::Index index;
    if (!map.getIndex(grid_map::Position(point.x(), point.y()), index)) {
      continue;
    }

    // Update all layers with the same pre-computed index (fold expression)
    (updaters.update(map, index, point), ...);
  }

  // Phase 3: Finalize all layers (compute derived statistics, etc.)
  (updaters.finalize(map), ...);
}

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_CORE_UPDATE_H
