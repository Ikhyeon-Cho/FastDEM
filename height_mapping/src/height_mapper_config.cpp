/*
 * height_mapper_config.cpp
 *
 * YAML configuration loading for HeightMapper::Config.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <yaml-cpp/yaml.h>

#include "height_mapping/height_mapper.h"

namespace height_mapping {

namespace {

template <typename T>
void loadIfExists(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

MappingMode parseMappingMode(const std::string& mode) {
  if (mode == "map_centric") return MappingMode::MAP_CENTRIC;
  return MappingMode::ROBOT_CENTRIC;  // default
}

HeightMapper::Config loadFromYaml(const YAML::Node& root) {
  HeightMapper::Config config;

  // Mapper section
  if (root["mapper"]) {
    const auto& mapper = root["mapper"];

    if (mapper["map"]) {
      const auto& m = mapper["map"];
      loadIfExists(m, "width", config.map.width);
      loadIfExists(m, "height", config.map.height);
      loadIfExists(m, "resolution", config.map.resolution);
      loadIfExists(m, "frame_id", config.map.frame_id);
    }

    // Mapping mode
    std::string mode_str;
    loadIfExists(mapper, "mode", mode_str);
    if (!mode_str.empty()) {
      config.mode = parseMappingMode(mode_str);
    }
  }

  // Algorithms section
  if (root["algorithms"]) {
    const auto& alg = root["algorithms"];

    // Voxel Filter
    if (alg["voxel_filter"]) {
      const auto& node = alg["voxel_filter"];
      loadIfExists(node, "voxel_size", config.voxel_filter.voxel_size);
      std::string method;
      loadIfExists(node, "reduction_method", method);
      if (method == "random") {
        config.voxel_filter.method =
            nanopcl::filters::VoxelGrid::Method::RANDOM;
      } else if (method == "first") {
        config.voxel_filter.method = nanopcl::filters::VoxelGrid::Method::FIRST;
      }
    }

    // Passthrough Filter
    if (alg["passthrough_filter"]) {
      const auto& node = alg["passthrough_filter"];
      loadIfExists(node, "x_min", config.passthrough_filter.x_min);
      loadIfExists(node, "x_max", config.passthrough_filter.x_max);
      loadIfExists(node, "y_min", config.passthrough_filter.y_min);
      loadIfExists(node, "y_max", config.passthrough_filter.y_max);
      loadIfExists(node, "z_min", config.passthrough_filter.z_min);
      loadIfExists(node, "z_max", config.passthrough_filter.z_max);
    }

    // Ground Segmentation
    if (alg["ground_segmentation"]) {
      const auto& node = alg["ground_segmentation"];
      loadIfExists(node, "method", config.ground_segmentation.method);
      // Grid params
      loadIfExists(node, "grid_resolution",
                   config.ground_segmentation.grid.grid_resolution);
      loadIfExists(node, "cell_percentile",
                   config.ground_segmentation.grid.cell_percentile);
      loadIfExists(node, "ground_thickness",
                   config.ground_segmentation.grid.ground_thickness);
      loadIfExists(node, "max_ground_height",
                   config.ground_segmentation.grid.max_ground_height);
      loadIfExists(node, "min_points_per_cell",
                   config.ground_segmentation.grid.min_points_per_cell);
      // Statistical params
      loadIfExists(node, "ground_percentile",
                   config.ground_segmentation.statistical.ground_percentile);
      loadIfExists(node, "noise_threshold",
                   config.ground_segmentation.statistical.noise_threshold);
      // Shared param
      if (node["ground_thickness"]) {
        config.ground_segmentation.statistical.ground_thickness =
            node["ground_thickness"].as<float>();
      }
    }

    // Raycasting
    if (alg["raycasting"]) {
      const auto& node = alg["raycasting"];
      float angle_deg = 0;
      if (node["max_ground_angle"]) {
        angle_deg = node["max_ground_angle"].as<float>();
        config.raycasting.max_ground_angle = angle_deg * M_PI / 180.0f;
      }
      loadIfExists(node, "correction_threshold",
                   config.raycasting.correction_threshold);
    }

    // Height Estimation
    if (alg["height_estimation"]) {
      const auto& node = alg["height_estimation"];
      loadIfExists(node, "estimator_type", config.height_estimation.type);
      // Kalman filter params
      loadIfExists(node, "kalman_process_noise",
                   config.height_estimation.kalman.process_noise);
      loadIfExists(node, "kalman_measurement_noise",
                   config.height_estimation.kalman.measurement_noise);
      loadIfExists(node, "kalman_initial_variance",
                   config.height_estimation.kalman.initial_variance);
      // Moving average params
      loadIfExists(node, "moving_average_alpha",
                   config.height_estimation.moving_average.alpha);
    }
  }

  return config;
}

}  // namespace

HeightMapper::Config HeightMapper::Config::load(const std::string& path) {
  try {
    return loadFromYaml(YAML::LoadFile(path));
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load config file '" + path +
                             "': " + e.what());
  }
}

}  // namespace height_mapping
