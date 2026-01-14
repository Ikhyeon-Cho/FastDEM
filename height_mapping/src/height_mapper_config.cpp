/*
 * height_mapper_config.cpp
 *
 * YAML configuration loading for HeightMapperConfig.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/height_mapper_config.h"

#include <yaml-cpp/yaml.h>

#include "detail/yaml_utils.h"

namespace height_mapping {

using detail::loadIfExists;

// =============================================================================
// VoxelFilterConfig
// =============================================================================

VoxelFilterConfig VoxelFilterConfig::fromYaml(const YAML::Node& node) {
  VoxelFilterConfig config;
  if (!node) return config;

  loadIfExists(node, "voxel_size", config.voxel_size);

  std::string method;
  loadIfExists(node, "reduction_method", method);
  if (method == "arbitrary") {
    config.method = VoxelMethod::ARBITRARY;
  } else if (method == "first") {
    config.method = VoxelMethod::FIRST;
  } else if (method == "centroid") {
    config.method = VoxelMethod::CENTROID;
  }
  return config;
}

// =============================================================================
// SpatialFilterConfig
// =============================================================================

SpatialFilterConfig SpatialFilterConfig::fromYaml(const YAML::Node& node) {
  SpatialFilterConfig config;
  if (!node) return config;

  loadIfExists(node, "z_min", config.z_min);
  loadIfExists(node, "z_max", config.z_max);
  loadIfExists(node, "range_min", config.range_min);
  loadIfExists(node, "range_max", config.range_max);
  return config;
}

// =============================================================================
// GroundSegmentationConfig
// =============================================================================

GroundSegmentationConfig GroundSegmentationConfig::fromYaml(const YAML::Node& node) {
  GroundSegmentationConfig config;
  if (!node) return config;

  loadIfExists(node, "enabled", config.enabled);
  loadIfExists(node, "grid_resolution", config.params.grid_resolution);
  loadIfExists(node, "cell_percentile", config.params.cell_percentile);
  loadIfExists(node, "ground_thickness", config.params.ground_thickness);
  loadIfExists(node, "max_ground_height", config.params.max_ground_height);
  loadIfExists(node, "min_points_per_cell", config.params.min_points_per_cell);
  return config;
}

// =============================================================================
// HeightEstimationConfig
// =============================================================================

HeightEstimationConfig HeightEstimationConfig::fromYaml(const YAML::Node& node) {
  HeightEstimationConfig config;
  if (!node) return config;

  loadIfExists(node, "estimator_type", config.type);
  // Kalman filter params
  loadIfExists(node, "kalman_process_noise", config.kalman.process_noise);
  loadIfExists(node, "kalman_measurement_noise", config.kalman.measurement_noise);
  loadIfExists(node, "kalman_initial_variance", config.kalman.initial_variance);
  // Moving average params
  loadIfExists(node, "moving_average_alpha", config.moving_average.alpha);
  return config;
}

// =============================================================================
// HeightMapperConfig
// =============================================================================

namespace {

MappingMode parseMappingMode(const std::string& mode) {
  if (mode == "map_centric") return MappingMode::MAP_CENTRIC;
  return MappingMode::ROBOT_CENTRIC;
}

HeightMapperConfig loadFromYaml(const YAML::Node& root) {
  HeightMapperConfig config;

  // Mapper section
  if (auto mapper = root["mapper"]) {
    if (auto map = mapper["map"]) {
      loadIfExists(map, "width", config.map.width);
      loadIfExists(map, "height", config.map.height);
      loadIfExists(map, "resolution", config.map.resolution);
      loadIfExists(map, "frame_id", config.map.frame_id);
    }

    std::string mode_str;
    loadIfExists(mapper, "mode", mode_str);
    if (!mode_str.empty()) {
      config.mode = parseMappingMode(mode_str);
    }
  }

  // Algorithms section - delegate to each Config's fromYaml()
  if (auto alg = root["algorithms"]) {
    config.voxel_filter = VoxelFilterConfig::fromYaml(alg["VoxelFilter"]);
    config.spatial_filter = SpatialFilterConfig::fromYaml(alg["PassthroughFilter"]);
    config.ground_segmentation = GroundSegmentationConfig::fromYaml(alg["GroundSegmentation"]);
    config.raycasting = algorithms::RaycastingConfig::fromYaml(alg["Raycasting"]);
    config.inpainting = algorithms::InpaintingConfig::fromYaml(alg["Inpainting"]);
    config.height_estimation = HeightEstimationConfig::fromYaml(alg["HeightEstimation"]);
  }

  return config;
}

}  // namespace

HeightMapperConfig HeightMapperConfig::load(const std::string& path) {
  try {
    return loadFromYaml(YAML::LoadFile(path));
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load config file '" + path +
                             "': " + e.what());
  }
}

}  // namespace height_mapping
