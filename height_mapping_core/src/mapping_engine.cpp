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
#include "pipeline_core/config_loader.h"
#include <chrono>
#include <iostream>

namespace height_mapping::core {

MappingEngine::MappingEngine(
    const EngineConfig &config,
    std::shared_ptr<ITransformProvider> transform_provider)
    : config_(config), transform_provider_(transform_provider) {

  initializeMap();
  setupPipeline();
  initialized_ = true;
}

// Factory method: Load pipeline config from YAML
std::unique_ptr<MappingEngine> MappingEngine::createFromFile(
    const std::string &config_file,
    std::shared_ptr<ITransformProvider> transform_provider) {

  // Create a simple engine config with the pipeline config file path
  EngineConfig config;
  config.pipeline_config_file = config_file;

  // TODO: Load map config from YAML if needed
  // For now, use defaults

  return std::make_unique<MappingEngine>(config, transform_provider);
}

void MappingEngine::initializeMap() {
  // Create and initialize the height map
  map_ = std::make_shared<height_map::HeightMap>();
  map_->initialize(config_.map.width, config_.map.height,
                   config_.map.resolution);
  map_->setFrameId(config_.map.frame_id);
}

void MappingEngine::setupPipeline() {
  // If we have a pipeline config file, load it
  if (!config_.pipeline_config_file.empty()) {
    mapping_pipeline_ = pipeline::ConfigLoader::createPipelineFromFile(config_.pipeline_config_file);
  } else {
    // Create a default pipeline with basic stages
    std::vector<pipeline::StageConfig> stage_configs;

    // Default transform stage
    pipeline::StageConfig transform_cfg;
    transform_cfg.type = "TransformStage";
    transform_cfg.params["target_frame"] = "map";
    stage_configs.push_back(transform_cfg);

    // Default voxel filter
    pipeline::StageConfig voxel_cfg;
    voxel_cfg.type = "VoxelFilterStage";
    voxel_cfg.params["voxel_size"] = "0.05";
    stage_configs.push_back(voxel_cfg);

    // Build pipeline
    mapping_pipeline_ = pipeline::PipelineBuilder()
                            .addStages(stage_configs)
                            .stopOnError(true)
                            .build();
  }
}

void MappingEngine::integrateCloud(const PointCloudXYZ &cloud) {
  MappingContext ctx;
  ctx.cloud() = cloud;
  integrateCloudImpl(ctx);
}

void MappingEngine::integrateCloud(PointCloudXYZ &&cloud) {
  MappingContext ctx;
  ctx.cloud() = std::move(cloud);
  integrateCloudImpl(ctx);
}

void MappingEngine::integrateCloudImpl(MappingContext &ctx) {
  if (!initialized_) {
    std::cerr << "[MappingEngine] Engine not initialized" << std::endl;
    return;
  }

  // Set the map reference
  ctx.replaceMap(map_);

  // Inject transform provider if available
  if (transform_provider_) {
    // The transform stage will look for this in the context
    // (This would require updating the TransformStage to check for a
    // provider)
    // For now, we'll just ensure the transform stage has access
  }

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
    if (config_.engine.auto_reset_on_error) {
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

std::shared_ptr<const PointCloudXYZ> MappingEngine::getProcessedCloud() const {
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

  // Reset statistics
  welford_stats_.count = 0;
  welford_stats_.mean = 0.0;
  welford_stats_.M2 = 0.0;
  welford_stats_.errors = 0;
}

MappingEngine::Statistics MappingEngine::getStatistics() const {
  Statistics stats;
  stats.clouds_processed = welford_stats_.count;
  stats.avg_processing_time_ms = welford_stats_.mean;
  if (welford_stats_.count > 1) {
    stats.variance_processing_time_ms = welford_stats_.M2 / (welford_stats_.count - 1);
  }
  stats.pipeline_errors = welford_stats_.errors;
  return stats;
}

void MappingEngine::updateStatistics(double duration_ms) {
  size_t n = ++welford_stats_.count;
  double old_mean = welford_stats_.mean.load();
  double delta = duration_ms - old_mean;
  double new_mean = old_mean + delta / n;
  welford_stats_.mean.store(new_mean);
  double delta2 = duration_ms - new_mean;
  double old_M2 = welford_stats_.M2.load();
  welford_stats_.M2.store(old_M2 + delta * delta2);
}

} // namespace height_mapping::core