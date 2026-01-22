/*
 * map.h
 *
 * Map geometry configuration types.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_MAP_H
#define HEIGHT_MAPPING_CONFIG_MAP_H

#include <string>

namespace height_mapping::config {

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

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_MAP_H
