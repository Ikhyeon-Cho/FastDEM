/*
 * height_mapper_config.h
 *
 * Configuration for HeightMapper
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_HEIGHT_MAPPER_CONFIG_H
#define HEIGHT_MAPPING_HEIGHT_MAPPER_CONFIG_H

#include <string>

// nanoPCL filters
#include <nanopcl/filters/downsample.hpp>

// Core algorithms
#include "height_mapping/algorithms/configs.h"
#include "height_mapping/estimators/kalman_filter.h"
#include "height_mapping/estimators/moving_average.h"
#include "height_mapping/segmenters/grid_ground_segmenter.h"
#include "height_mapping/types.h"

// Forward declaration for YAML loading
namespace YAML { class Node; }

namespace height_mapping {

using npcl::filters::VoxelMethod;

enum class MappingMode { ROBOT_CENTRIC, MAP_CENTRIC };

/// Voxel filter configuration
struct VoxelFilterConfig {
  float voxel_size = 0.1f;
  VoxelMethod method = VoxelMethod::CENTROID;

  static VoxelFilterConfig fromYaml(const YAML::Node& node);
};

/// Spatial filter configuration
struct SpatialFilterConfig {
  float z_min = -std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::max();
  float range_min = 0.0f;
  float range_max = std::numeric_limits<float>::max();

  static SpatialFilterConfig fromYaml(const YAML::Node& node);
};

/// Ground segmentation configuration
struct GroundSegmentationConfig {
  bool enabled = true;
  segmenters::GridGroundSegmenter::Parameters params;

  static GroundSegmentationConfig fromYaml(const YAML::Node& node);
};

/// Height estimation configuration
struct HeightEstimationConfig {
  std::string type = "incremental_mean";
  estimators::KalmanFilter::Parameters kalman;
  estimators::MovingAverage::Parameters moving_average;

  static HeightEstimationConfig fromYaml(const YAML::Node& node);
};

/**
 * @brief Configuration for HeightMapper
 */
struct HeightMapperConfig {
  HeightMap::Config map;
  MappingMode mode = MappingMode::ROBOT_CENTRIC;

  // Algorithm Configs
  VoxelFilterConfig voxel_filter;
  SpatialFilterConfig spatial_filter;
  GroundSegmentationConfig ground_segmentation;
  algorithms::RaycastingConfig raycasting;
  algorithms::InpaintingConfig inpainting;
  HeightEstimationConfig height_estimation;

  /// Load configuration from YAML file
  static HeightMapperConfig load(const std::string& path);
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_HEIGHT_MAPPER_CONFIG_H
