/*
 * configs.cpp
 *
 * YAML loading implementations for algorithm configurations.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/algorithms/configs.h"
#include "detail/yaml_utils.h"

namespace height_mapping::algorithms {

using detail::loadIfExists;

PersistenceConfig PersistenceConfig::fromYaml(const YAML::Node& node) {
  PersistenceConfig config;
  if (!node) return config;

  loadIfExists(node, "persistence_max_count", config.max_count);
  loadIfExists(node, "persistence_step_add", config.step_add);
  loadIfExists(node, "persistence_step_sub", config.step_sub);
  return config;
}

RaycastingConfig RaycastingConfig::fromYaml(const YAML::Node& node) {
  RaycastingConfig config;
  if (!node) return config;

  loadIfExists(node, "enable_correction", config.enable_correction);
  loadIfExists(node, "correction_threshold", config.correction_threshold);
  loadIfExists(node, "min_ray_distance", config.min_ray_distance);
  config.persistence = PersistenceConfig::fromYaml(node);
  return config;
}

InpaintingConfig InpaintingConfig::fromYaml(const YAML::Node& node) {
  InpaintingConfig config;
  if (!node) return config;

  loadIfExists(node, "enabled", config.enabled);
  loadIfExists(node, "max_iterations", config.max_iterations);
  loadIfExists(node, "min_valid_neighbors", config.min_valid_neighbors);
  return config;
}

}  // namespace height_mapping::algorithms
