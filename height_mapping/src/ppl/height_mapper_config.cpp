/*
 * height_mapper_config.cpp
 *
 * YAML configuration loading for ppl::HeightMapper::Config.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/ppl/height_mapper.h"

#include <yaml-cpp/yaml.h>

namespace height_mapping::ppl {

namespace {

template <typename T>
void loadIfExists(const YAML::Node& node, const std::string& key, T& value) {
  if (node[key]) {
    value = node[key].as<T>();
  }
}

HeightMapper::Config loadFromYaml(const YAML::Node& yaml,
                                  const std::string& file_path) {
  HeightMapper::Config config;

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

  // Store the file path for pipeline loading
  config.pipeline_config_path = file_path;

  return config;
}

}  // namespace

HeightMapper::Config HeightMapper::Config::load(const std::string& path) {
  try {
    return loadFromYaml(YAML::LoadFile(path), path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to load ppl config file '" + path +
                             "': " + e.what());
  }
}

}  // namespace height_mapping::ppl
