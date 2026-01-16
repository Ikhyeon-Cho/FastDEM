/*
 * config.cpp
 *
 * YAML configuration loading for MappingConfig.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <yaml-cpp/yaml.h>

#include "height_mapping/api/config.h"

namespace height_mapping {
namespace {

template <typename T>
void load(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

config::MappingMode parseMode(const std::string& mode) {
  if (mode == "global") return config::MappingMode::GLOBAL;
  return config::MappingMode::LOCAL;
}

config::VoxelMethod parseVoxelMethod(const std::string& method) {
  if (method == "arbitrary") return config::VoxelMethod::ARBITRARY;
  if (method == "first") return config::VoxelMethod::FIRST;
  return config::VoxelMethod::CENTROID;
}

MappingConfig parseConfig(const YAML::Node& root) {
  MappingConfig cfg;

  // Map
  if (auto n = root["map"]) {
    load(n, "width", cfg.map.width);
    load(n, "height", cfg.map.height);
    load(n, "resolution", cfg.map.resolution);
    load(n, "frame_id", cfg.map.frame_id);
    std::string mode;
    load(n, "mode", mode);
    if (!mode.empty()) cfg.map.mode = parseMode(mode);
  }

  // Voxel filter
  if (auto n = root["voxel_filter"]) {
    load(n, "voxel_size", cfg.voxel_filter.voxel_size);
    std::string method;
    load(n, "method", method);
    if (!method.empty()) cfg.voxel_filter.method = parseVoxelMethod(method);
  }

  // Spatial filter
  if (auto n = root["spatial_filter"]) {
    load(n, "z_min", cfg.spatial_filter.z_min);
    load(n, "z_max", cfg.spatial_filter.z_max);
    load(n, "range_min", cfg.spatial_filter.range_min);
    load(n, "range_max", cfg.spatial_filter.range_max);
  }

  // Raycasting
  if (auto n = root["raycasting"]) {
    load(n, "enabled", cfg.raycasting.enabled);
    load(n, "threshold", cfg.raycasting.threshold);
    load(n, "min_distance", cfg.raycasting.min_distance);
    if (auto p = n["persistence"]) {
      load(p, "max_count", cfg.raycasting.persistence.max_count);
      load(p, "step_add", cfg.raycasting.persistence.step_add);
      load(p, "step_sub", cfg.raycasting.persistence.step_sub);
    }
  }

  // Inpainting
  if (auto n = root["inpainting"]) {
    load(n, "enabled", cfg.inpainting.enabled);
    load(n, "max_iterations", cfg.inpainting.max_iterations);
    load(n, "min_valid_neighbors", cfg.inpainting.min_valid_neighbors);
  }

  // Estimation
  if (auto n = root["estimation"]) {
    load(n, "type", cfg.estimation.type);
    if (auto k = n["kalman"]) {
      load(k, "process_noise", cfg.estimation.kalman.process_noise);
      load(k, "measurement_noise", cfg.estimation.kalman.measurement_noise);
      load(k, "initial_variance", cfg.estimation.kalman.initial_variance);
    }
    if (auto m = n["moving_average"]) {
      load(m, "alpha", cfg.estimation.moving_average.alpha);
    }
  }

  return cfg;
}

}  // namespace

MappingConfig MappingConfig::load(const std::string& path) {
  try {
    return parseConfig(YAML::LoadFile(path));
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load config: " + path + " - " +
                             e.what());
  }
}

}  // namespace height_mapping
