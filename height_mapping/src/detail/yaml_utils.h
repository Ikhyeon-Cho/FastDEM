/*
 * yaml_utils.h
 *
 * Internal YAML parsing utilities shared between mappers.
 * This file is NOT part of the public API.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_DETAIL_YAML_UTILS_H
#define HEIGHT_MAPPING_DETAIL_YAML_UTILS_H

#include <yaml-cpp/yaml.h>

namespace height_mapping::detail {

/// Load value from YAML node if key exists, otherwise keep default
template <typename T>
void loadIfExists(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

}  // namespace height_mapping::detail

#endif  // HEIGHT_MAPPING_DETAIL_YAML_UTILS_H
