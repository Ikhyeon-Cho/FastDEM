/*
 * pipeline_builder.h
 *
 * PUBLIC API - Pipeline construction and configuration.
 *
 * This header provides the builder pattern for constructing pipelines.
 * Users can build pipelines in three ways:
 * 1. From configuration files (recommended)
 * 2. From Config objects
 * 3. Programmatically using the builder pattern
 *
 * Example usage:
 *   // Method 1: From file (most common)
 *   auto pipeline = pipeline::PipelineBuilder::fromFile("config.yaml");
 *
 *   // Method 2: From Config
 *   auto config = pipeline::Config::fromFile("config.yaml");
 *   auto pipeline = pipeline::PipelineBuilder::fromConfig(config);
 *
 *   // Method 3: Programmatic construction
 *   pipeline::PipelineBuilder builder;
 *   builder.addStage("VoxelFilter", {{"voxel_size", "0.1"}})
 *          .addStage("Transform", {{"target_frame", "map"}})
 *          .stopOnError(true);
 *   auto pipeline = builder.build();
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_PIPELINE_BUILDER_H
#define PIPELINE_CORE_PIPELINE_BUILDER_H

#include "pipeline_core/config.h"
#include "pipeline_core/pipeline.h"
#include "pipeline_core/stage.h"
#include "pipeline_core/stage_registry.h"
#include <memory>
#include <string>
#include <vector>

namespace pipeline {

/**
 * @brief Builder for constructing pipelines with Factory support
 */
class PipelineBuilder {
public:
  PipelineBuilder() { pipeline_ = std::make_unique<Pipeline>(); }

  virtual ~PipelineBuilder() = default;

  // Method 1: Direct stage creation (type-safe)
  PipelineBuilder &addStage(std::unique_ptr<Stage> stage) {
    if (stage) {
      pipeline_->addStage(std::move(stage));
    }
    return *this;
  }

  // Method 2: Factory-based creation with config
  PipelineBuilder &addStage(const StageConfig &config) {
    auto stage = StageRegistry::create(config.name);

    // Apply configuration
    stage->configure(config.params);
    stage->setEnabled(config.enabled);

    return addStage(std::move(stage));
  }

  // Method 3: Simple factory creation with name and params
  PipelineBuilder &
  addStage(const std::string &name,
           const std::map<std::string, std::string> &params = {}) {
    StageConfig config;
    config.name = name;
    config.params = params;
    config.enabled = true;
    return addStage(config);
  }

  // Add multiple stages from config list
  PipelineBuilder &addStages(const std::vector<StageConfig> &stages) {
    for (const auto &stage_config : stages) {
      addStage(stage_config);
    }
    return *this;
  }

  PipelineBuilder &stopOnError(bool stop = true) {
    pipeline_->setStopOnError(stop);
    return *this;
  }

  PipelineBuilder &enableStage(const std::string &name, bool enable = true) {
    pipeline_->enableStage(name, enable);
    return *this;
  }

  virtual std::unique_ptr<Pipeline> build() { return std::move(pipeline_); }

  // Static factory methods for creating pipelines from configuration
  static std::unique_ptr<Pipeline> fromFile(const std::string &filename) {
    auto config = Config::fromFile(filename);
    return fromConfig(config);
  }

  static std::unique_ptr<Pipeline> fromConfig(const Config &config) {
    PipelineBuilder builder;
    builder.addStages(config.stages);
    builder.stopOnError(config.stop_on_error);

    return builder.build();
  }

protected:
  std::unique_ptr<Pipeline> pipeline_;
};

} // namespace pipeline

#endif // PIPELINE_CORE_PIPELINE_BUILDER_H