/*
 * stage_registry.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_STAGE_REGISTRY_H
#define PIPELINE_CORE_STAGE_REGISTRY_H

#include "pipeline_core/stage.h"
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace pipeline {

class StageRegistry {
public:
  using Creator = std::function<Stage::Ptr()>;

  // Register a stage type
  template <typename T> static void registerStage(const std::string &name) {
    getRegistry()[name] = []() { return std::make_unique<T>(); };
  }

  // Create a stage by type name
  static Stage::Ptr create(const std::string &type) {
    auto &registry = getRegistry();
    auto it = registry.find(type);
    if (it != registry.end()) {
      return it->second();
    }
    throw std::runtime_error("Unknown stage type: " + type);
  }

  // Check if a stage type is registered
  static bool hasStage(const std::string &type) {
    return getRegistry().find(type) != getRegistry().end();
  }

  // Get list of registered stage types
  static std::vector<std::string> getRegisteredTypes() {
    std::vector<std::string> types;
    for (const auto &pair : getRegistry()) {
      types.push_back(pair.first);
    }
    return types;
  }

private:
  static std::map<std::string, Creator> &getRegistry() {
    static std::map<std::string, Creator> registry;
    return registry;
  }
};

// Macro for automatic stage registration
#define REGISTER_STAGE(Class)                                                  \
  namespace {                                                                  \
  static bool _register_##Class = []() {                                       \
    pipeline::StageRegistry::registerStage<Class>(#Class);                     \
    return true;                                                               \
  }();                                                                         \
  }

} // namespace pipeline

#endif // PIPELINE_CORE_STAGE_REGISTRY_H