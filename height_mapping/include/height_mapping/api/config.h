/*
 * config.h
 *
 * Configuration for Mapper (simple, fixed pipeline).
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_API_CONFIG_H
#define HEIGHT_MAPPING_API_CONFIG_H

#include <string>

#include "height_mapping/config/types.h"

namespace height_mapping {

/**
 * @brief Configuration for Mapper (fixed pipeline).
 *
 * Simple configuration for the Mapper class.
 * For dynamic pipeline configuration, see lab::Mapper::Config.
 *
 * @code
 * auto config = MappingConfig::load("config.yaml");
 * Mapper mapper(config, extrinsics, pose);
 * @endcode
 */
struct MappingConfig {
  config::Map map;
  config::VoxelFilter voxel_filter;
  config::SpatialFilter spatial_filter;
  config::Raycasting raycasting;
  config::Inpainting inpainting;
  config::Estimation estimation;

  /**
   * @brief Load configuration from YAML file.
   * @param path Path to YAML configuration file
   * @return Loaded configuration
   * @throws std::runtime_error If file cannot be loaded or parsed
   */
  static MappingConfig load(const std::string& path);
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_API_CONFIG_H
