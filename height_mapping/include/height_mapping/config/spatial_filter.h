/*
 * spatial_filter.h
 *
 * Spatial filtering configuration (z-range and distance bounds).
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_SPATIAL_FILTER_H
#define HEIGHT_MAPPING_CONFIG_SPATIAL_FILTER_H

#include <limits>

namespace height_mapping::config {

/**
 * @brief Spatial filtering configuration (z-range and distance bounds).
 */
struct SpatialFilter {
  float z_min = -std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::max();
  float range_min = 0.0f;
  float range_max = std::numeric_limits<float>::max();
};

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_SPATIAL_FILTER_H
