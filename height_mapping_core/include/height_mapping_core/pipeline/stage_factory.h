/*
 * stage_factory.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/config/pipeline_config.h"
#include "height_mapping_core/pipeline/stage.h"
#include "height_mapping_core/pipeline/stages/transform_stage.h"
#include <memory>
#include <string>
#include <functional>
#include <any>
#include <unordered_map>
#include <vector>

namespace height_mapping::core {

// Factory for creating pipeline stages based on configuration
class StageFactory {
public:
  // Stage creation configuration
  struct StageConfig {
    std::string type;  // "transform", "voxel_filter", etc.
    std::unordered_map<std::string, std::any> parameters;

    // Helper methods for type-safe parameter access
    template<typename T>
    T getParam(const std::string& key, const T& default_value = T{}) const {
      auto it = parameters.find(key);
      if (it != parameters.end()) {
        try {
          return std::any_cast<T>(it->second);
        } catch (...) {
          return default_value;
        }
      }
      return default_value;
    }
  };

  // Register a stage creator function
  using StageCreator = std::function<std::unique_ptr<IPipelineStage>(
      const StageConfig&, std::shared_ptr<ITransformProvider>)>;

  static void registerStage(const std::string& type, StageCreator creator);

  // Create a stage from configuration
  static std::unique_ptr<IPipelineStage> create(
      const StageConfig& config,
      std::shared_ptr<ITransformProvider> transform_provider = nullptr);

private:
  static std::unordered_map<std::string, StageCreator>& getRegistry();
};

// Helper macro for registering stages
#define REGISTER_STAGE(Type, Creator) \
  static bool _registered_##Type = []() { \
    StageFactory::registerStage(#Type, Creator); \
    return true; \
  }();

} // namespace height_mapping::core