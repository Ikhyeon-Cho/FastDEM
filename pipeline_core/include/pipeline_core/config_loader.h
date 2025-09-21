/*
 * config_loader.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_CONFIG_LOADER_H
#define PIPELINE_CORE_CONFIG_LOADER_H

#include "pipeline_core/config.h"
#include "pipeline_core/pipeline_builder.h"
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>

namespace pipeline {

class ConfigLoader {
public:
  // YAML parsing with yaml-cpp library
  static PipelineConfig loadYaml(const std::string &filename) {
    PipelineConfig config;

    try {
      YAML::Node root = YAML::LoadFile(filename);

      // Load pipeline settings
      if (root["pipeline"]) {
        auto pipeline_node = root["pipeline"];
        if (pipeline_node["stop_on_error"]) {
          config.stop_on_error = pipeline_node["stop_on_error"].as<bool>();
        }
        // Load global params if present
        if (pipeline_node["global_params"]) {
          for (const auto &param : pipeline_node["global_params"]) {
            config.global_params[param.first.as<std::string>()] =
                param.second.as<std::string>();
          }
        }
      }

      // Load stages
      if (root["stages"]) {
        for (const auto &stage_node : root["stages"]) {
          StageConfig stage;

          // Required: type
          stage.type = stage_node["type"].as<std::string>();

          // Optional: name
          if (stage_node["name"]) {
            stage.name = stage_node["name"].as<std::string>();
          }

          // Optional: enabled (default: true)
          if (stage_node["enabled"]) {
            stage.enabled = stage_node["enabled"].as<bool>();
          }

          // Optional: params
          if (stage_node["params"]) {
            for (const auto &param : stage_node["params"]) {
              stage.params[param.first.as<std::string>()] =
                  param.second.as<std::string>();
            }
          }

          config.stages.push_back(stage);
        }
      }

    } catch (const YAML::Exception &e) {
      throw std::runtime_error("Failed to parse YAML config: " +
                              std::string(e.what()));
    }

    return config;
  }

  // Create pipeline from YAML config file
  static std::unique_ptr<Pipeline> createPipelineFromFile(const std::string &filename) {
    auto config = loadYaml(filename);

    // Build pipeline from config
    PipelineBuilder builder;
    builder.addStages(config.stages);
    builder.stopOnError(config.stop_on_error);

    return builder.build();
  }

  // Create pipeline from YAML string
  static std::unique_ptr<Pipeline> createPipelineFromString(const std::string &yaml_str) {
    PipelineConfig config;

    try {
      YAML::Node root = YAML::Load(yaml_str);

      // Same parsing logic as loadYaml
      if (root["pipeline"]) {
        auto pipeline_node = root["pipeline"];
        if (pipeline_node["stop_on_error"]) {
          config.stop_on_error = pipeline_node["stop_on_error"].as<bool>();
        }
      }

      if (root["stages"]) {
        for (const auto &stage_node : root["stages"]) {
          StageConfig stage;
          stage.type = stage_node["type"].as<std::string>();

          if (stage_node["name"]) {
            stage.name = stage_node["name"].as<std::string>();
          }
          if (stage_node["enabled"]) {
            stage.enabled = stage_node["enabled"].as<bool>();
          }
          if (stage_node["params"]) {
            for (const auto &param : stage_node["params"]) {
              stage.params[param.first.as<std::string>()] =
                  param.second.as<std::string>();
            }
          }

          config.stages.push_back(stage);
        }
      }
    } catch (const YAML::Exception &e) {
      throw std::runtime_error("Failed to parse YAML string: " +
                              std::string(e.what()));
    }

    // Build pipeline
    PipelineBuilder builder;
    builder.addStages(config.stages);
    builder.stopOnError(config.stop_on_error);

    return builder.build();
  }
};

} // namespace pipeline

#endif // PIPELINE_CORE_CONFIG_LOADER_H