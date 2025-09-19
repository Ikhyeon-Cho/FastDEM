/*
 * config_loader.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/config/config_loader.h"
#include <fstream>
#include <stdexcept>

namespace height_mapping::core {

EngineConfig ConfigLoader::loadEngineConfig(const std::string &yaml_file) {
  if (!validateFile(yaml_file)) {
    throw std::runtime_error("Configuration file not found or not readable: " +
                             yaml_file);
  }

  YAML::Node config_node = YAML::LoadFile(yaml_file);
  EngineConfig config;

  // Parse map configuration
  if (config_node["map"]) {
    parseMapConfig(config_node["map"], config.map);
  }

  // Parse engine configuration
  if (config_node["engine"]) {
    parseEngineConfig(config_node["engine"], config.engine);
  }

  // Parse pipeline configuration embedded in engine config
  if (config_node["pipeline"]) {
    config.pipeline = loadPipelineConfig(yaml_file);
  }

  return config;
}

PipelineConfig ConfigLoader::loadPipelineConfig(const std::string &yaml_file) {
  if (!validateFile(yaml_file)) {
    throw std::runtime_error("Configuration file not found or not readable: " +
                             yaml_file);
  }

  YAML::Node config_node = YAML::LoadFile(yaml_file);
  PipelineConfig config;

  // Get pipeline section
  if (!config_node["pipeline"]) {
    throw std::runtime_error(
        "No 'pipeline' section found in configuration file");
  }

  YAML::Node pipeline_node = config_node["pipeline"];

  // Parse stages enablement
  if (pipeline_node["stages"]) {
    YAML::Node stages = pipeline_node["stages"];
    parsePipelineStages(stages, config);
  }

  // Parse filter configuration
  if (pipeline_node["filters"]) {
    parseFilterConfig(pipeline_node["filters"], config.filters);
  }

  // Parse estimator configuration
  if (pipeline_node["estimator"]) {
    parseEstimatorConfig(pipeline_node["estimator"], config.estimator);
  }

  // Parse transform configuration
  if (pipeline_node["transform"]) {
    config.transform.target_frame =
        pipeline_node["transform"]["target_frame"].as<std::string>("map");
  }

  // Parse map update configuration
  if (pipeline_node["map_update"]) {
    YAML::Node map_update = pipeline_node["map_update"];

    std::string mode = map_update["mode"].as<std::string>("continuous");
    if (mode == "continuous") {
      config.map_update.mode = PipelineConfig::MapUpdate::CONTINUOUS;
    } else if (mode == "threshold") {
      config.map_update.mode = PipelineConfig::MapUpdate::THRESHOLD;
    } else if (mode == "centered") {
      config.map_update.mode = PipelineConfig::MapUpdate::CENTERED;
    }

    config.map_update.threshold = map_update["threshold"].as<double>(1.0);
    config.map_update.robot_frame =
        map_update["robot_frame"].as<std::string>("base_link");
    // map_frame is derived from transform.target_frame
  }

  // Parse behavior
  if (pipeline_node["behavior"]) {
    config.behavior.stop_on_error =
        pipeline_node["behavior"]["stop_on_error"].as<bool>(true);
    config.behavior.enable_profiling =
        pipeline_node["behavior"]["enable_profiling"].as<bool>(false);
  }

  return config;
}

std::pair<EngineConfig, PipelineConfig>
ConfigLoader::loadConfig(const std::string &yaml_file) {
  EngineConfig engine_config = loadEngineConfig(yaml_file);
  // Pipeline config is already loaded as part of engine config
  return {engine_config, engine_config.pipeline};
}

bool ConfigLoader::validateFile(const std::string &yaml_file) {
  std::ifstream file(yaml_file);
  return file.good();
}

void ConfigLoader::parseMapConfig(const YAML::Node &node,
                                  EngineConfig::Map &config) {
  config.width = node["width"].as<double>(config.width);
  config.height = node["height"].as<double>(config.height);
  config.resolution = node["resolution"].as<double>(config.resolution);
  config.frame_id = node["frame_id"].as<std::string>(config.frame_id);
}

void ConfigLoader::parsePipelineStages(const YAML::Node &node,
                                       PipelineConfig &config) {
  config.stages.transform = node["transform"].as<bool>(true);
  config.stages.voxel_filter = node["voxel_filter"].as<bool>(true);
  config.stages.passthrough_filter = node["passthrough_filter"].as<bool>(true);
  config.stages.height_estimation = node["height_estimation"].as<bool>(true);
  config.stages.map_update = node["map_update"].as<bool>(true);
}

void ConfigLoader::parseFilterConfig(const YAML::Node &node,
                                     PipelineConfig::Filters &filters) {
  // Voxel filter
  if (node["voxel"]) {
    filters.voxel_size = node["voxel"]["size"].as<double>(0.05);
  }

  // Passthrough filter
  if (node["passthrough"]) {
    auto &pass = node["passthrough"];
    filters.x_min = pass["x_min"].as<double>(-100.0);
    filters.x_max = pass["x_max"].as<double>(100.0);
    filters.y_min = pass["y_min"].as<double>(-100.0);
    filters.y_max = pass["y_max"].as<double>(100.0);
    filters.z_min = pass["z_min"].as<double>(-2.0);
    filters.z_max = pass["z_max"].as<double>(5.0);
  }
}

void ConfigLoader::parseEstimatorConfig(const YAML::Node &node,
                                        PipelineConfig::Estimator &estimator) {
  std::string type = node["type"].as<std::string>("kalman");

  if (type == "kalman") {
    estimator.type = PipelineConfig::Estimator::KALMAN;
  } else if (type == "mean") {
    estimator.type = PipelineConfig::Estimator::MEAN;
  } else if (type == "moving_average") {
    estimator.type = PipelineConfig::Estimator::MOVING_AVERAGE;
  }

  // Kalman parameters
  if (node["kalman"]) {
    estimator.kalman_process_noise =
        node["kalman"]["process_noise"].as<double>(0.01);
    estimator.kalman_measurement_noise =
        node["kalman"]["measurement_noise"].as<double>(0.1);
  }

  // Moving average parameters
  if (node["moving_average"]) {
    estimator.moving_average_window =
        node["moving_average"]["window"].as<int>(5);
  }
}

void ConfigLoader::parseEngineConfig(const YAML::Node &node,
                                     EngineConfig::Engine &engine) {
  engine.thread_safe = node["thread_safe"].as<bool>(true);
  engine.auto_reset_on_error = node["auto_reset_on_error"].as<bool>(false);
  engine.max_queue_size = node["max_queue_size"].as<size_t>(10);
  engine.enable_statistics = node["enable_statistics"].as<bool>(false);
}

} // namespace height_mapping::core