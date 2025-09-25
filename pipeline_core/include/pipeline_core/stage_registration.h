/*
 * stage_registration.h
 *
 * IMPLEMENTATION FILE ONLY - Include this header only in .cpp files.
 *
 * This header provides the REGISTER_STAGE macro for automatic stage
 *registration. Domain-specific developers should:
 *   1. Define their stage class in a .h file (inheriting from pipeline::Stage)
 *   2. Include this header ONLY in their .cpp file
 *   3. Call REGISTER_STAGE(YourStageName) in the .cpp file
 *
 * Example usage (in .cpp file only):
 *   #include "my_stage.h"
 *   #include "pipeline_core/stage_registration.h"
 *
 *   // Implementation of MyStage methods...
 *
 *   REGISTER_STAGE(MyStage);  // Auto-registers with factory
 *
 * This allows your stage to be created by name in configuration files:
 *   stages:
 *     - name: "MyStage"
 *       params:
 *         key: value
 *
 * WARNING: The StageRegistry class below is INTERNAL - do not use directly.
 *          Only use the REGISTER_STAGE macro.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_STAGE_REGISTRATION_H
#define PIPELINE_CORE_STAGE_REGISTRATION_H

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

#endif // PIPELINE_CORE_STAGE_REGISTRATION_H