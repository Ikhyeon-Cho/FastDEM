/*
 * median_filter.h
 *
 * Median filter for removing spike noise from height map layers.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_OPS_MEDIAN_FILTER_H
#define HEIGHT_MAPPING_OPS_MEDIAN_FILTER_H

#include "height_mapping/config/median_filter.h"
#include "height_mapping/core/map.h"

namespace height_mapping::ops {

/**
 * @brief Applies median filter to specified layers (in-place).
 *
 * Removes spike noise while preserving edges.
 * Each layer in target_layers is processed independently.
 *
 * @param map Height map to process
 * @param config Median filter configuration
 */
void applyMedianFilter(HeightMap& map, const config::MedianFilter& config);

}  // namespace height_mapping::ops

#endif  // HEIGHT_MAPPING_OPS_MEDIAN_FILTER_H
