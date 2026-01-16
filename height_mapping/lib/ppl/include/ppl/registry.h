/*
 * registry.h - Stage registry and auto-registration macro
 *
 * Factory pattern implementation for creating stages by name.
 * Uses static registration to avoid manual registry updates.
 *
 * Usage:
 *   // At the end of your stage implementation file:
 *   PPL_REGISTER_STAGE(MyContext, MyStage, "MyStage")
 *
 *   // Then in YAML:
 *   params:
 *     MyStage:
 *       param1: value1
 *
 * Created on: Dec 2024
 * Author: Ikhyeon Cho
 * Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 * Email: tre0430@korea.ac.kr
 */

#ifndef PPL_REGISTRY_H
#define PPL_REGISTRY_H

#include "stage.h"

#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace ppl {

/**
 * @brief Stage factory and registry
 *
 * Template singleton that maps type names to stage creators.
 * Uses Meyers singleton to avoid static initialization order issues.
 *
 * @tparam TContext User-defined context type
 */
template <typename TContext>
class Registry {
public:
  using StagePtr = std::shared_ptr<Stage<TContext>>;
  using CreatorFunc = std::function<StagePtr()>;

  /**
   * @brief Register a stage type with its creator function
   *
   * @param type_name Name used in YAML 'type' field
   * @param creator Function that creates a new stage instance
   * @return true if registration succeeded, false if duplicate
   */
  static bool add(const std::string& type_name, CreatorFunc creator) {
    auto& map = getMap();
    if (map.find(type_name) != map.end()) {
      return false;  // Duplicate registration
    }
    map[type_name] = creator;
    return true;
  }

  /**
   * @brief Create a stage instance by type name
   *
   * @param type_name Name of the stage type (from YAML 'type' field)
   * @return Shared pointer to newly created stage
   * @throws std::runtime_error if type_name is not registered
   */
  static StagePtr create(const std::string& type_name) {
    auto& map = getMap();
    auto it = map.find(type_name);
    if (it == map.end()) {
      throw std::runtime_error("[ppl] Unknown stage type: " + type_name);
    }
    return it->second();
  }

  /**
   * @brief Check if a stage type is registered
   */
  static bool has(const std::string& type_name) {
    return getMap().find(type_name) != getMap().end();
  }

private:
  // Meyers singleton to avoid static initialization order fiasco
  static std::map<std::string, CreatorFunc>& getMap() {
    static std::map<std::string, CreatorFunc> map;
    return map;
  }
};

}  // namespace ppl

/**
 * @brief Macro for automatic stage registration
 *
 * Place this at the end of your stage implementation file (.cpp).
 * The stage will be automatically registered when the library loads.
 *
 * @param ContextType The context type used by the pipeline
 * @param ClassName The stage class to register
 * @param TypeName String name used in YAML 'type' field
 *
 * Example:
 *   PPL_REGISTER_STAGE(MappingContext, VoxelFilterStage, "VoxelFilter")
 */
#define PPL_REGISTER_STAGE(ContextType, ClassName, TypeName)                   \
  namespace {                                                                  \
  struct PPL_Registrar_##ClassName {                                           \
    PPL_Registrar_##ClassName() {                                              \
      try {                                                                    \
        bool success = ppl::Registry<ContextType>::add(                        \
            TypeName, []() { return std::make_shared<ClassName>(); });         \
        if (!success) {                                                        \
          std::cerr << "[ppl] FATAL: Duplicate stage registration: "           \
                    << TypeName << std::endl;                                  \
          std::exit(1);                                                        \
        }                                                                      \
      } catch (const std::exception& e) {                                      \
        std::cerr << "[ppl] FATAL: Stage registration failed for " << TypeName \
                  << ": " << e.what() << std::endl;                            \
        std::exit(1);                                                          \
      }                                                                        \
    }                                                                          \
  };                                                                           \
  static PPL_Registrar_##ClassName ppl_registrar_instance_##ClassName          \
      __attribute__((used));                                                   \
  }

#endif  // PPL_REGISTRY_H
