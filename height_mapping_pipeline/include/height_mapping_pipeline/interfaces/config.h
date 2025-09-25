#ifndef HEIGHT_MAPPING_PIPELINE_CONFIG_H
#define HEIGHT_MAPPING_PIPELINE_CONFIG_H

#include "pipeline_core/config.h"
#include <string>
#include <yaml-cpp/yaml.h>

namespace height_mapping::mapping {

struct Config {
  struct Map {
    double width{15.0};
    double height{15.0};
    double resolution{0.1};
    std::string frame_id{"map"};
    bool robot_centric{true};
    std::string update_mode{"continuous"};
    double update_threshold{1.0};
  } map;

  struct Engine {
    bool thread_safe{true};
    bool reset_on_error{false};
    bool enable_statistics{false};
    bool verbose{false};
  } engine;

  pipeline::Config pipeline;

  static Config fromFile(const std::string &file_path);
  static Config fromYAML(const YAML::Node &yaml);
};

inline Config Config::fromFile(const std::string &file_path) {
  if (file_path.empty()) {
    return Config();
  }
  return fromYAML(YAML::LoadFile(file_path));
}

inline Config Config::fromYAML(const YAML::Node &yaml) {
  Config config;

  if (!yaml)
    return config;

  auto readValue = [](const YAML::Node &node, const std::string &key,
                      auto &target) {
    if (node && node[key]) {
      target = node[key].as<std::decay_t<decltype(target)>>();
    }
  };

  if (auto engine = yaml["engine"]) {
    if (auto map = engine["map"]) {
      readValue(map, "width", config.map.width);
      readValue(map, "height", config.map.height);
      readValue(map, "resolution", config.map.resolution);
      readValue(map, "frame_id", config.map.frame_id);
      readValue(map, "robot_centric", config.map.robot_centric);
      readValue(map, "update_mode", config.map.update_mode);
      readValue(map, "update_threshold", config.map.update_threshold);
    }

    if (auto runtime = engine["runtime"]) {
      readValue(runtime, "thread_safe", config.engine.thread_safe);
      readValue(runtime, "reset_on_error", config.engine.reset_on_error);
      readValue(runtime, "enable_statistics", config.engine.enable_statistics);
      readValue(runtime, "verbose", config.engine.verbose);
    }
  }

  config.pipeline = pipeline::Config::fromYAML(yaml);
  return config;
}

} // namespace height_mapping::mapping

#endif // HEIGHT_MAPPING_PIPELINE_CONFIG_H