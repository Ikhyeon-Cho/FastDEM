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

#include <cmath>

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

  // Spatial filter
  if (auto n = root["spatial_filter"]) {
    load(n, "z_min", cfg.spatial_filter.z_min);
    load(n, "z_max", cfg.spatial_filter.z_max);
    load(n, "range_min", cfg.spatial_filter.range_min);
    load(n, "range_max", cfg.spatial_filter.range_max);
  }

  // Raycasting (temporal voting for ghost removal)
  if (auto n = root["raycasting"]) {
    load(n, "enabled", cfg.raycasting.enabled);
    load(n, "endpoint_margin", cfg.raycasting.endpoint_margin);
    load(n, "ray_height_margin", cfg.raycasting.ray_height_margin);
    load(n, "dynamic_height_threshold", cfg.raycasting.dynamic_height_threshold);
    load(n, "vote_threshold", cfg.raycasting.vote_threshold);
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
    load(n, "alpha", cfg.estimation.alpha);
    if (auto k = n["kalman"]) {
      load(k, "min_variance", cfg.estimation.kalman.min_variance);
      load(k, "max_variance", cfg.estimation.kalman.max_variance);
      load(k, "process_noise", cfg.estimation.kalman.process_noise);
    }
    if (auto p = n["p2"]) {
      load(p, "dn0", cfg.estimation.p2.dn0);
      load(p, "dn1", cfg.estimation.p2.dn1);
      load(p, "dn2", cfg.estimation.p2.dn2);
      load(p, "dn3", cfg.estimation.p2.dn3);
      load(p, "dn4", cfg.estimation.p2.dn4);
      load(p, "elevation_marker", cfg.estimation.p2.elevation_marker);
      load(p, "max_sample_count", cfg.estimation.p2.max_sample_count);
    }
  }

  // Sensor model
  if (auto n = root["sensor_model"]) {
    load(n, "type", cfg.sensor_model.type);
    // Treat deprecated aliases
    if (cfg.sensor_model.type == "none") {
      cfg.sensor_model.type = "constant";
    } else if (cfg.sensor_model.type == "laser") {
      cfg.sensor_model.type = "lidar";  // "laser" → "lidar"
    }
    load(n, "range_noise", cfg.sensor_model.range_noise);
    load(n, "angular_noise", cfg.sensor_model.angular_noise);
    load(n, "default_uncertainty", cfg.sensor_model.default_uncertainty);
  }

  // Outlier rejection
  if (auto n = root["outlier_rejection"]) {
    load(n, "enabled", cfg.outlier_rejection.enabled);
    load(n, "sigma_threshold", cfg.outlier_rejection.sigma_threshold);
    load(n, "min_uncertainty", cfg.outlier_rejection.min_uncertainty);
  }

  // Spatial fusion (bilateral filter + weighted ECDF)
  if (auto n = root["spatial_fusion"]) {
    load(n, "enabled", cfg.spatial_fusion.enabled);
    load(n, "search_radius", cfg.spatial_fusion.search_radius);
    load(n, "spatial_sigma", cfg.spatial_fusion.spatial_sigma);
    load(n, "quantile_lower", cfg.spatial_fusion.quantile_lower);
    load(n, "quantile_upper", cfg.spatial_fusion.quantile_upper);
    load(n, "min_valid_neighbors", cfg.spatial_fusion.min_valid_neighbors);
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
