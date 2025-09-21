/*
 * pipeline_builder.h
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
    auto stage = StageRegistry::create(config.type);

    // Apply configuration
    stage->configure(config.params);
    stage->setEnabled(config.enabled);

    // Set custom name if provided
    if (!config.name.empty()) {
      // Note: Stage name is set in constructor, might need adjustment
    }

    return addStage(std::move(stage));
  }

  // Method 3: Simple factory creation with type and params
  PipelineBuilder &
  addStage(const std::string &type,
           const std::map<std::string, std::string> &params = {}) {
    StageConfig config;
    config.type = type;
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

protected:
  std::unique_ptr<Pipeline> pipeline_;
};

} // namespace pipeline

#endif // PIPELINE_CORE_PIPELINE_BUILDER_H