/*
 * config.h
 *
 * PUBLIC API - Pipeline configuration structures and parsing.
 *
 * This header provides configuration data structures and YAML parsing
 * functionality for pipeline setup. Users typically use this for:
 * - Loading pipeline configurations from YAML files
 * - Programmatically creating pipeline configurations
 *
 * Example usage:
 *   // Load from file
 *   auto config = pipeline::Config::fromFile("pipeline.yaml");
 *
 *   // Parse from YAML node
 *   YAML::Node node = ...;
 *   auto config = pipeline::Config::fromYAML(node);
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_CONFIG_H
#define PIPELINE_CORE_CONFIG_H

#include <map>
#include <string>
#include <type_traits>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace pipeline {

// Configuration for a single stage
struct StageConfig {
  std::string name;    // Stage name for factory creation and identification
  bool enabled = true; // Stage enabled/disabled flag
  std::map<std::string, std::string> params; // Stage-specific parameters

  // Helper methods for type conversion
  template <typename T>
  T getParam(const std::string &key, const T &default_value) const {
    auto it = params.find(key);
    if (it == params.end()) {
      return default_value;
    }

    if constexpr (std::is_same_v<T, bool>) {
      return it->second == "true" || it->second == "1";
    } else if constexpr (std::is_same_v<T, int>) {
      return std::stoi(it->second);
    } else if constexpr (std::is_same_v<T, float>) {
      return std::stof(it->second);
    } else if constexpr (std::is_same_v<T, double>) {
      return std::stod(it->second);
    } else {
      return it->second;
    }
  }
};

// Configuration for entire pipeline
struct Config {
  std::vector<StageConfig> stages; // List of stages to create
  bool stop_on_error = true;       // Pipeline error handling behavior
  std::map<std::string, std::string>
      global_params; // Global parameters for all stages

  // Parse configuration from YAML node with flexible structure
  static Config fromYAML(const YAML::Node &root);

  // Load configuration from YAML file
  static Config fromFile(const std::string &filename);
};

// Inline implementations
inline Config Config::fromYAML(const YAML::Node &root) {
  Config config;

  // Determine where to look for pipeline config
  // Priority: root["pipeline"] > root itself
  YAML::Node pipeline_node;
  YAML::Node stages_node;

  // Case 1: Check for nested pipeline.stages
  if (root["pipeline"] && root["pipeline"]["stages"]) {
    pipeline_node = root["pipeline"];
    stages_node = root["pipeline"]["stages"];
  }
  // Case 2: Check for direct stages under root
  else if (root["stages"]) {
    pipeline_node = root; // Settings at root level
    stages_node = root["stages"];
  }
  // Case 3: Mixed - pipeline settings but stages at root (rare but supported)
  else if (root["pipeline"] && root["stages"]) {
    pipeline_node = root["pipeline"];
    stages_node = root["stages"];
  }
  // Case 4: No stages found - return empty config
  else {
    return config;
  }

  // Parse pipeline settings
  if (pipeline_node) {
    // stop_on_error setting
    if (pipeline_node["stop_on_error"]) {
      config.stop_on_error = pipeline_node["stop_on_error"].as<bool>();
    }

    // Global parameters (rarely used but supported)
    if (pipeline_node["global_params"]) {
      for (const auto &param : pipeline_node["global_params"]) {
        config.global_params[param.first.as<std::string>()] =
            param.second.as<std::string>();
      }
    }
  }

  // Parse stages
  if (stages_node) {
    for (const auto &stage_node : stages_node) {
      StageConfig stage;

      // Required: name field
      if (!stage_node["name"]) {
        throw std::runtime_error("Stage missing required 'name' field");
      }
      stage.name = stage_node["name"].as<std::string>();

      // Optional: enabled field (default: true)
      if (stage_node["enabled"]) {
        stage.enabled = stage_node["enabled"].as<bool>();
      }

      // Optional: params field
      if (stage_node["params"]) {
        // Convert all param values to strings regardless of YAML type
        for (const auto &param : stage_node["params"]) {
          std::string key = param.first.as<std::string>();
          // Convert any scalar type to string
          // This handles: numbers (0.05), strings ("value"), bools (true)
          stage.params[key] = param.second.as<std::string>();
        }
      }

      config.stages.push_back(stage);
    }
  }

  return config;
}

inline Config Config::fromFile(const std::string &filename) {
  try {
    YAML::Node root = YAML::LoadFile(filename);
    return fromYAML(root);
  } catch (const YAML::Exception &e) {
    throw std::runtime_error("Failed to load YAML file: " +
                             std::string(e.what()));
  }
}

} // namespace pipeline

#endif // PIPELINE_CORE_CONFIG_H