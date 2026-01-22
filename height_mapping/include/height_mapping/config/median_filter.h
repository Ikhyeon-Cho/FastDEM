/*
 * median_filter.h
 *
 * Median filter configuration for spike noise removal.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_MEDIAN_FILTER_H
#define HEIGHT_MAPPING_CONFIG_MEDIAN_FILTER_H

#include <string>
#include <vector>

namespace height_mapping::config {

/**
 * @brief Median filter configuration for spike noise removal.
 */
struct MedianFilter {
  bool enabled = false;
  int kernel_size = 3;
  int min_valid_neighbors = 5;
  std::vector<std::string> target_layers = {"elevation_max"};
};

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_MEDIAN_FILTER_H
