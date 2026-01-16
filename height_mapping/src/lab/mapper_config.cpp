/*
 * mapper_config.cpp
 *
 * YAML configuration loading for ppl::Mapper::Config.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <yaml-cpp/yaml.h>

#include "height_mapping/lab/mapper.h"

namespace height_mapping::lab {

namespace {

template <typename T>
void loadIfExists(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

Mapper::Config loadFromYaml(const YAML::Node& yaml) {
  Mapper::Config config;

  // Parse mapper-specific settings
  if (auto mapper = yaml["mapper"]) {
    if (auto map = mapper["map"]) {
      loadIfExists(map, "width", config.map.width);
      loadIfExists(map, "height", config.map.height);
      loadIfExists(map, "resolution", config.map.resolution);
      loadIfExists(map, "frame_id", config.map.frame_id);
    }

    loadIfExists(mapper, "enable_debug", config.enable_debug);
  }

  // Parse pipeline profiling settings
  if (auto pipeline = yaml["pipeline"]) {
    loadIfExists(pipeline, "enable_profiling", config.enable_profiling);
    loadIfExists(pipeline, "profile_interval", config.profile_interval);
  }

  // Store loaded YAML for pipeline loading
  config.pipeline = yaml;

  return config;
}

}  // namespace

Mapper::Config Mapper::Config::load(const std::string& path) {
  try {
    return loadFromYaml(YAML::LoadFile(path));
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load ppl config file '" + path +
                             "': " + e.what());
  }
}

}  // namespace height_mapping::lab
