/*
 * mapping_engine.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/mapping_engine.h"
#include "logger/logger.h"
#include "pipeline_core/pipeline_builder.h"

namespace height_mapping::core {

constexpr const char *ENGINE_NAME = "MappingEngine";

MappingEngine::MappingEngine(
    std::shared_ptr<ITransformProvider> transform_provider,
    const Config &config)
    : config_(config), transform_provider_(transform_provider) {

  initializeMap();
  setupMappingPipeline();
}

void MappingEngine::initializeMap() {
  map_ = std::make_shared<height_map::HeightMap>();
  map_->initialize(config_.map.width, config_.map.height,
                   config_.map.resolution);
  map_->setFrameId(config_.map.frame_id);
}

void MappingEngine::setupMappingPipeline() {
  // Use default pipeline if config is not available (from YAML)
  if (config_.pipeline.stages.empty()) {
    LOG_DEBUG(ENGINE_NAME,
              "No pipeline configuration found, using default pipeline");
    setupDefaultPipeline();
    return;
  }

  mapping_pipeline_ = pipeline::PipelineBuilder::fromConfig(config_.pipeline);
  LOG_DEBUG(ENGINE_NAME, "Pipeline created from configuration with ",
            config_.pipeline.stages.size(), " stages");

  // Initialize profiler (optional component)
  profiler_ =
      std::make_unique<pipeline::PipelineProfiler>(mapping_pipeline_.get());
}

void MappingEngine::setupDefaultPipeline() {
  std::vector<pipeline::StageConfig> stages;

  // 1. Transform to map frame (essential for mapping)
  pipeline::StageConfig transform_stage;
  transform_stage.name = "TransformCloud";
  transform_stage.params["target_frame"] = "map";
  stages.push_back(transform_stage);

  // 2. Height estimation (core functionality)
  pipeline::StageConfig height_stage;
  height_stage.name = "HeightEstimation";
  height_stage.params["estimator_type"] = "incremental_mean";
  stages.push_back(height_stage);

  // Create pipeline config and build
  pipeline::Config default_config;
  default_config.stages = stages;
  default_config.stop_on_error = true;

  mapping_pipeline_ = pipeline::PipelineBuilder::fromConfig(default_config);

  // Initialize profiler for default pipeline too
  profiler_ =
      std::make_unique<pipeline::PipelineProfiler>(mapping_pipeline_.get());
}

void MappingEngine::registerCloud(std::shared_ptr<PointCloud> cloud) {
  // Validation
  if (!cloud || cloud->empty() || cloud->frameId().empty()) {
    LOG_ERROR(ENGINE_NAME, "Invalid point cloud provided");
    return;
  }

  // Create context with cloud and map
  MappingContext ctx(cloud, map_);
  ctx.setService(transform_provider_);

  // Process through pipeline (with optional profiling)
  try {
    if (benchmark_enabled_ && profiler_) {
      profiler_->processWithProfiling(ctx);
    } else {
      mapping_pipeline_->process(ctx); // Clean path, zero overhead
    }

    // Update processed cloud reference
    if (config_.engine.thread_safe) {
      std::lock_guard<std::shared_mutex> lock(cloud_mutex_);
      last_processed_cloud_ = ctx.cloudPtr();
    } else {
      last_processed_cloud_ = ctx.cloudPtr();
    }
  } catch (const std::exception &e) {
    LOG_ERROR_THROTTLE(1.0, ENGINE_NAME,
                       "Pipeline processing failed: ", e.what());
    if (config_.engine.reset_on_error) {
      reset();
    }
  }
}

std::shared_ptr<const height_map::HeightMap>
MappingEngine::getHeightMap() const {
  if (config_.engine.thread_safe) {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    return map_;
  }
  return map_;
}

std::shared_ptr<const PointCloud> MappingEngine::getProcessedCloud() const {
  if (config_.engine.thread_safe) {
    std::shared_lock<std::shared_mutex> lock(cloud_mutex_);
    return last_processed_cloud_;
  }
  return last_processed_cloud_;
}

void MappingEngine::reset() {
  if (config_.engine.thread_safe) {
    std::lock_guard<std::shared_mutex> lock(map_mutex_);
    map_->clear();
  } else {
    map_->clear();
  }
}

void MappingEngine::setBenchmarkEnabled(bool enable) {
  benchmark_enabled_ = enable;
  if (profiler_) {
    profiler_->setAutoprint(enable);
  }
}

void MappingEngine::setBenchmarkInterval(size_t interval) {
  if (profiler_) {
    profiler_->setPrintInterval(interval);
  }
}

} // namespace height_mapping::core