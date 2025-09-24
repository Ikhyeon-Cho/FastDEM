/*
 * mapping_engine.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/mapping_engine.h"
#include "pipeline_core/pipeline_builder.h"

#include <iostream>

namespace height_mapping::core {

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
    std::cout << "No pipeline configuration found, using default pipeline"
              << std::endl;
    setupDefaultPipeline();
    return;
  }

  mapping_pipeline_ = pipeline::PipelineBuilder::fromConfig(config_.pipeline);
  std::cout << "Pipeline created from configuration with "
            << config_.pipeline.stages.size() << " stages" << std::endl;
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
}

void MappingEngine::registerCloud(std::shared_ptr<PointCloud> cloud) {
  // Validation
  if (!cloud || cloud->empty() || cloud->frameId().empty()) {
    std::cerr << "[MappingEngine] Invalid point cloud provided" << std::endl;
    return;
  }

  // Create context with cloud and map
  MappingContext ctx(cloud, map_);
  ctx.setService(transform_provider_);

  // Process through pipeline
  try {
    mapping_pipeline_->process(ctx);

    // Update processed cloud reference
    if (config_.engine.thread_safe) {
      std::lock_guard<std::shared_mutex> lock(cloud_mutex_);
      last_processed_cloud_ = ctx.cloudPtr();
    } else {
      last_processed_cloud_ = ctx.cloudPtr();
    }
  } catch (const std::exception &e) {
    std::cerr << "[MappingEngine] Pipeline processing failed: " << e.what()
              << std::endl;
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

} // namespace height_mapping::core