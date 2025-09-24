/*
 * config.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_CONFIG_H
#define HEIGHT_MAPPING_CORE_CONFIG_H

#include "pipeline_core/config.h"
#include <string>
#include <yaml-cpp/yaml.h>

namespace height_mapping::core {

// Forward declarations
struct Config;
inline Config parseConfig(const YAML::Node &yaml);

struct Config {

  struct Map {
    double width{15.0};     // meters
    double height{15.0};    // meters
    double resolution{0.1}; // meters per cell
    std::string frame_id{"map"};
    bool robot_centric{true};              // Move map with robot
    std::string update_mode{"continuous"}; // continuous, centered, fixed
    double update_threshold{1.0};          // meters (for threshold mode)
  } map;

  struct Engine {
    bool thread_safe{true};
    bool reset_on_error{false};
    bool enable_statistics{false};
    bool verbose{false};
  } engine;

  pipeline::Config pipeline;

  static Config fromFile(const std::string &file_path) {
    if (file_path.empty()) {
      return Config(); // Return default configuration
    }

    auto yaml = YAML::LoadFile(file_path);
    return parseConfig(yaml);
  }
};

inline Config parseConfig(const YAML::Node &yaml) {
  Config config;

  if (!yaml)
    return config;

  // Helper lambda for safe value reading
  auto readValue = [](const YAML::Node &node, const std::string &key,
                      auto &target) {
    if (node && node[key]) {
      target = node[key].as<std::decay_t<decltype(target)>>();
    }
  };

  // Parse engine section from YAML
  if (yaml["engine"]) {
    auto engine = yaml["engine"];

    // Parse map configuration from engine.map
    if (engine["map"]) {
      auto map = engine["map"];
      readValue(map, "width", config.map.width);
      readValue(map, "height", config.map.height);
      readValue(map, "resolution", config.map.resolution);
      readValue(map, "frame_id", config.map.frame_id);
      readValue(map, "robot_centric", config.map.robot_centric);
      readValue(map, "update_mode", config.map.update_mode);
      readValue(map, "update_threshold", config.map.update_threshold);
    }

    // Parse runtime configuration from engine.runtime
    if (engine["runtime"]) {
      auto runtime = engine["runtime"];
      readValue(runtime, "thread_safe", config.engine.thread_safe);
      readValue(runtime, "reset_on_error", config.engine.reset_on_error);
      readValue(runtime, "enable_statistics", config.engine.enable_statistics);
      readValue(runtime, "verbose", config.engine.verbose);
    }
  }

  // Parse pipeline configuration using Config's built-in parser
  config.pipeline = pipeline::Config::fromYAML(yaml);

  return config;
}

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_CONFIG_H