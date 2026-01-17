/*
 * types.h
 *
 * Shared configuration types for height mapping.
 * Pure data structures without parsing logic.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_TYPES_H
#define HEIGHT_MAPPING_CONFIG_TYPES_H

#include <limits>
#include <nanopcl/filters/downsample.hpp>
#include <string>

namespace height_mapping::config {

using npcl::filters::VoxelMethod;

/**
 * @brief Mapping mode for height map origin behavior.
 */
enum class MappingMode {
  LOCAL,  ///< Map origin follows robot (robot-centric)
  GLOBAL  ///< Map origin is fixed (world frame)
};

/**
 * @brief Map geometry configuration.
 */
struct Map {
  float width = 15.0f;
  float height = 15.0f;
  float resolution = 0.1f;
  std::string frame_id = "map";
  MappingMode mode = MappingMode::LOCAL;
};

/**
 * @brief Voxel grid downsampling configuration.
 */
struct VoxelFilter {
  float voxel_size = 0.05f;
  VoxelMethod method = VoxelMethod::CENTROID;
};

/**
 * @brief Spatial filtering configuration (z-range and distance bounds).
 */
struct SpatialFilter {
  float z_min = -std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::max();
  float range_min = 0.0f;
  float range_max = std::numeric_limits<float>::max();
};

/**
 * @brief Kalman filter parameters for height estimation.
 */
struct Kalman {
  float process_noise = 0.01f;
  float measurement_noise = 0.1f;
  float initial_variance = 1.0f;
};

/**
 * @brief Moving average parameters for height estimation.
 */
struct MovingAverage {
  float alpha = 0.3f;
};

/**
 * @brief Height estimation configuration.
 */
struct Estimation {
  /// Algorithm type: incremental_mean, kalman_filter, moving_average
  std::string type = "incremental_mean";
  Kalman kalman;
  MovingAverage moving_average;
};

/**
 * @brief Persistence (obstacle durability) configuration.
 */
struct Persistence {
  float max_count = 30.0f;
  float step_add = 3.0f;
  float step_sub = 1.0f;
};

/**
 * @brief Raycasting configuration for ghost removal.
 */
struct Raycasting {
  bool enabled = false;
  float threshold = 0.05f;
  float min_distance = 0.5f;
  Persistence persistence;
};

/**
 * @brief Inpainting configuration for hole filling.
 */
struct Inpainting {
  bool enabled = false;
  int max_iterations = 3;
  int min_valid_neighbors = 2;
};

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_TYPES_H
