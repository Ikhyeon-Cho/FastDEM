/*
 * mapper.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/mapper.h"

#include "flowpipe/pipeline_builder.h"
#include "height_mapping/pipeline/mapping_context.h"
#include "logger/logger.h"

namespace height_mapping {

constexpr const char *LABEL = "HeightMapper";

HeightMapper::HeightMapper(TransformLookup::Ptr tf, Config cfg)
    : cfg_(cfg), tf_tree_(tf), map_(initMap(cfg_)) {
  setupMappingPipeline();
}

// TODO: HeightMap from Config -> Need HeightMap Config class.
HeightMap::Ptr HeightMapper::initMap(const Config &cfg) {
  auto map = std::make_shared<HeightMap>();
  map->configure(cfg.map.width, cfg.map.height, cfg.map.resolution);
  map->setFrameId(cfg.map.frame_id);
  return map;
}

void HeightMapper::setupMappingPipeline() {
  // Use default pipeline if config is not available (from YAML)
  if (cfg_.pipeline.stages.empty()) {
    LOG_DEBUG(LABEL, "No pipeline configuration found, using default pipeline");
    setupDefaultPipeline();
    return;
  }

  mapping_pipeline_ = flowpipe::PipelineBuilder::fromConfig(cfg_.pipeline);
  LOG_DEBUG(LABEL, "Pipeline created from configuration with ",
            cfg_.pipeline.stages.size(), " stages");

  // Initialize profiler (optional component)
  profiler_ =
      std::make_unique<flowpipe::PipelineProfiler>(mapping_pipeline_.get());
}

void HeightMapper::setupDefaultPipeline() {
  std::vector<flowpipe::StageConfig> stages;

  // 1. Transform to map frame (essential for mapping)
  flowpipe::StageConfig transform_stage;
  transform_stage.name = "TransformCloud";
  transform_stage.params["target_frame"] = "map";
  stages.push_back(transform_stage);

  // 2. Height estimation (core functionality)
  flowpipe::StageConfig height_stage;
  height_stage.name = "HeightEstimation";
  height_stage.params["estimator_type"] = "incremental_mean";
  stages.push_back(height_stage);

  // Create pipeline config and build
  flowpipe::Config default_config;
  default_config.stages = stages;
  default_config.stop_on_error = true;

  mapping_pipeline_ = flowpipe::PipelineBuilder::fromConfig(default_config);

  // Initialize profiler for default pipeline too
  profiler_ =
      std::make_unique<flowpipe::PipelineProfiler>(mapping_pipeline_.get());
}

void HeightMapper::integrate(std::shared_ptr<PointCloud> cloud) {
  // Validation
  if (!cloud || cloud->empty() || cloud->frameId().empty()) {
    LOG_ERROR(LABEL, "Invalid point cloud provided");
    return;
  }

  // Create context with cloud and map
  MappingContext ctx(cloud, map_);
  ctx.setService(tf_tree_);

  // Process through pipeline (with optional profiling)
  try {
    if (benchmark_enabled_ && profiler_) {
      profiler_->processWithProfiling(ctx);
    } else {
      mapping_pipeline_->process(ctx);  // Clean path, zero overhead
    }

    // Update processed cloud reference
    if (cfg_.engine.thread_safe) {
      std::lock_guard<std::shared_mutex> lock(cloud_mutex_);
      last_processed_cloud_ = ctx.cloudPtr();
    } else {
      last_processed_cloud_ = ctx.cloudPtr();
    }
  } catch (const std::exception &e) {
    LOG_ERROR_THROTTLE(1.0, LABEL, "Pipeline processing failed: ", e.what());
    if (cfg_.engine.reset_on_error) {
      reset();
    }
  }
}

HeightMap::ConstPtr HeightMapper::getHeightMap() const {
  if (cfg_.engine.thread_safe) {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    return map_;
  }
  return map_;
}

std::shared_ptr<const PointCloud> HeightMapper::getScanProcessed() const {
  if (cfg_.engine.thread_safe) {
    std::shared_lock<std::shared_mutex> lock(cloud_mutex_);
    return last_processed_cloud_;
  }
  return last_processed_cloud_;
}

void HeightMapper::reset() {
  if (cfg_.engine.thread_safe) {
    std::lock_guard<std::shared_mutex> lock(map_mutex_);
    map_->clear();
  } else {
    map_->clear();
  }
}

void HeightMapper::setBenchmarkEnabled(bool enable) {
  benchmark_enabled_ = enable;
  if (profiler_) {
    profiler_->setAutoprint(enable);
  }
}

void HeightMapper::setBenchmarkInterval(size_t interval) {
  if (profiler_) {
    profiler_->setPrintInterval(interval);
  }
}

}  // namespace height_mapping