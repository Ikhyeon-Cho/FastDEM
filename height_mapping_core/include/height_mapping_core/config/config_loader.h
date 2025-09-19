/*
 * config_loader.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_CONFIG_LOADER_H
#define HEIGHT_MAPPING_CORE_CONFIG_LOADER_H

#include "height_mapping_core/engine/height_mapping_engine.h" // Has EngineConfig and PipelineConfig
#include <string>
#include <yaml-cpp/yaml.h>

namespace height_mapping::core {

// Pure YAML configuration loader - NO ROS dependencies
class ConfigLoader {
public:
  // Load engine configuration from YAML file
  static EngineConfig loadEngineConfig(const std::string &yaml_file);

  // Load pipeline configuration from YAML file
  static PipelineConfig loadPipelineConfig(const std::string &yaml_file);

  // Load combined configuration
  static std::pair<EngineConfig, PipelineConfig>
  loadConfig(const std::string &yaml_file);

  // Validate configuration file exists and is readable
  static bool validateFile(const std::string &yaml_file);

private:
  // Helper functions for parsing specific sections
  static void parseMapConfig(const YAML::Node &node, EngineConfig::Map &config);
  static void parsePipelineStages(const YAML::Node &node,
                                  PipelineConfig &config);
  static void parseFilterConfig(const YAML::Node &node,
                                PipelineConfig::Filters &filters);
  static void parseEstimatorConfig(const YAML::Node &node,
                                   PipelineConfig::Estimator &estimator);
  static void parseEngineConfig(const YAML::Node &node,
                                EngineConfig::Engine &engine);
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_CONFIG_LOADER_H