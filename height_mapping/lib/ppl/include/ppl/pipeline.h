/*
 * pipeline.h - Pipeline engine for ppl (Pipeline Library)
 *
 * Loads pipeline configuration from YAML and executes stages sequentially.
 * Supports separated flow (pipeline.stages) and config (algorithms) sections.
 *
 * YAML Structure:
 *   pipeline:
 *     stages:
 *       - MoveOrigin
 *       - PreVoxel
 *       - HeightEstimation
 *
 *   algorithms:
 *     MoveOrigin:
 *       type: MoveOrigin    # optional if name == type
 *       z_offset: 1.5
 *     PreVoxel:
 *       type: VoxelFilter   # reuse same class with different params
 *       voxel_size: 0.1
 *
 * Created on: Dec 2024
 * Author: Ikhyeon Cho
 * Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 * Email: tre0430@korea.ac.kr
 */

#ifndef PPL_PIPELINE_H
#define PPL_PIPELINE_H

#include "detail/log.h"
#include "registry.h"
#include "stage.h"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ppl {

/**
 * @brief Pipeline engine that loads and executes stages
 *
 * @tparam TContext User-defined context type (data bus)
 */
template <typename TContext>
class Pipeline {
public:
  using StagePtr = std::shared_ptr<Stage<TContext>>;

  Pipeline() = default;
  ~Pipeline() = default;

  /**
   * @brief Load pipeline from YAML file
   *
   * @param yaml_path Path to YAML configuration file
   */
  void load(const std::string& yaml_path) {
    try {
      YAML::Node root = YAML::LoadFile(yaml_path);
      load(root);
    } catch (const YAML::Exception& e) {
      throw std::runtime_error("[ppl] Failed to load YAML file: " + yaml_path +
                               " - " + e.what());
    }
  }

  /**
   * @brief Load pipeline from YAML node
   *
   * @param root Root YAML node containing 'pipeline' and 'algorithms' sections
   */
  void load(const YAML::Node& root) {
    stages_.clear();

    // Validate required sections
    if (!root["pipeline"] || !root["pipeline"]["stages"]) {
      throw std::runtime_error("[ppl] Missing 'pipeline.stages' in YAML");
    }

    const auto& stage_names = root["pipeline"]["stages"];

    // Type safety: ensure stages is a sequence (list)
    if (!stage_names.IsSequence()) {
      throw std::runtime_error(
          "[ppl] 'pipeline.stages' must be a list (YAML sequence)");
    }

    const auto& algorithms =
        root["algorithms"];  // Optional: parameter storage

    for (const auto& node : stage_names) {
      std::string instance_name = node.as<std::string>();
      std::string type_name = instance_name;  // Default: name is type
      YAML::Node config_node;

      // Look up configuration in algorithms section
      if (algorithms && algorithms[instance_name]) {
        YAML::Node algo_config = algorithms[instance_name];

        // Use 'type' field if specified, otherwise use instance_name
        if (algo_config["type"]) {
          type_name = algo_config["type"].as<std::string>();
        }
        config_node = algo_config;
      }

      // Create stage via registry
      try {
        auto stage = Registry<TContext>::create(type_name);
        stage->setName(instance_name);
        stage->configure(config_node);
        stages_.push_back(stage);

        PPL_LOG_DEBUG("[ppl] Loaded stage: {} (type: {})", instance_name,
                      type_name);
      } catch (const std::exception& e) {
        PPL_LOG_ERROR("[ppl] Failed to create stage '{}': {}", instance_name,
                      e.what());
        throw;
      }
    }

    PPL_LOG_INFO("[ppl] Pipeline loaded with {} stages", stages_.size());
  }

  /**
   * @brief Execute all stages sequentially
   *
   * Catches exceptions from individual stages for graceful shutdown.
   *
   * @param context Shared pointer to the context (data bus)
   * @return true if all stages succeeded, false if any stage failed
   */
  bool run(const std::shared_ptr<TContext>& context) {
    for (auto& stage : stages_) {
      try {
        if (!stage->process(context)) {
          PPL_LOG_WARN("[ppl] Stage '{}' returned false, stopping pipeline",
                       stage->name());
          return false;
        }
      } catch (const std::exception& e) {
        PPL_LOG_ERROR("[ppl] Stage '{}' threw exception: {}", stage->name(),
                      e.what());
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Get the list of stages (for profiler access)
   */
  const std::vector<StagePtr>& stages() const { return stages_; }

  /**
   * @brief Get the number of stages
   */
  size_t size() const { return stages_.size(); }

  /**
   * @brief Check if pipeline is empty
   */
  bool empty() const { return stages_.empty(); }

private:
  std::vector<StagePtr> stages_;
};

}  // namespace ppl

#endif  // PPL_PIPELINE_H
