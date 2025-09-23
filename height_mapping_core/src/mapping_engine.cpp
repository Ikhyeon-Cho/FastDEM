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

#include <chrono>
#include <filesystem>
#include <fstream>
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
  // Use pipeline config if available (from YAML)
  if (!config_.pipeline.stages.empty()) {
    mapping_pipeline_ = pipeline::PipelineBuilder::fromConfig(config_.pipeline);
    std::cout << "Pipeline created from configuration with "
              << config_.pipeline.stages.size() << " stages" << std::endl;
    return;
  }

  // Fall back to default hardcoded pipeline
  std::cout << "Using default pipeline (no stages in config)" << std::endl;
  setupDefaultPipeline();
}

void MappingEngine::setupDefaultPipeline() {
  std::vector<pipeline::StageConfig> stage_configs;

  // Update map origin to follow robot (if robot-centric mapping is enabled)
  if (config_.map.robot_centric) {
    pipeline::StageConfig map_origin_cfg;
    map_origin_cfg.name = "MapOriginUpdate";
    map_origin_cfg.params["update_mode"] = config_.map.update_mode;
    map_origin_cfg.params["update_threshold"] =
        std::to_string(config_.map.update_threshold);
    map_origin_cfg.params["robot_frame"] = "base_link";
    map_origin_cfg.params["map_frame"] = config_.map.frame_id;
    stage_configs.push_back(map_origin_cfg);
  }

  // Transform to base frame for filtering
  pipeline::StageConfig cloud_to_base;
  cloud_to_base.name = "PointCloudTransform";
  cloud_to_base.params["target_frame"] = "base_link";
  stage_configs.push_back(cloud_to_base);

  // Voxel filter for downsampling (after transform to base frame)
  pipeline::StageConfig voxel_cfg;
  voxel_cfg.name = "VoxelFilter";
  voxel_cfg.params["voxel_size"] = "0.05";           // 5cm voxels
  voxel_cfg.params["reduction_method"] = "centroid"; // Use centroid method
  stage_configs.push_back(voxel_cfg);

  // Filter local pointcloud (in base_link frame)
  pipeline::StageConfig passthrough_cfg;
  passthrough_cfg.name = "PassthroughFilter";
  passthrough_cfg.params["x_min"] = "-10.0";
  passthrough_cfg.params["x_max"] = "10.0";
  passthrough_cfg.params["y_min"] = "-10.0";
  passthrough_cfg.params["y_max"] = "10.0";
  passthrough_cfg.params["z_min"] = "-0.1";
  passthrough_cfg.params["z_max"] = "2.0";
  stage_configs.push_back(passthrough_cfg);

  // Transform to map frame for height estimation
  pipeline::StageConfig transform_cfg;
  transform_cfg.name = "PointCloudTransform";
  transform_cfg.params["target_frame"] = config_.map.frame_id;
  stage_configs.push_back(transform_cfg);

  // // Raycasting for height correction (before estimation)
  // pipeline::StageConfig raycast_cfg;
  // raycast_cfg.name = "Raycasting";
  // raycast_cfg.params["max_ground_angle"] = "-5.0";     // degrees
  // raycast_cfg.params["correction_threshold"] = "0.02"; // 2cm
  // raycast_cfg.params["enable_correction"] = "true";
  // stage_configs.push_back(raycast_cfg);

  // Height estimation
  pipeline::StageConfig height_est_cfg;
  height_est_cfg.name = "HeightEstimation";
  height_est_cfg.params["estimator_type"] = "incremental_mean";
  stage_configs.push_back(height_est_cfg);

  // Build mapping pipeline using Config
  pipeline::Config default_pipeline_config;
  default_pipeline_config.stages = stage_configs;
  default_pipeline_config.stop_on_error = true;
  mapping_pipeline_ =
      pipeline::PipelineBuilder::fromConfig(default_pipeline_config);
}

void MappingEngine::integrateCloud(std::shared_ptr<PointCloud> cloud) {
  // Validation
  if (!cloud) {
    std::cerr << "[MappingEngine] Null cloud pointer provided" << std::endl;
    return;
  }
  if (!mapping_pipeline_) {
    std::cerr << "[MappingEngine] Mapping pipeline not set up" << std::endl;
    return;
  }
  if (cloud->empty()) {
    std::cerr << "[MappingEngine] Empty point cloud, skipping integration"
              << std::endl;
    return;
  }
  if (cloud->frameId().empty()) {
    std::cerr << "[MappingEngine] Point cloud has empty frame_id, skipping"
              << std::endl;
    return;
  }

  // Create context with cloud and map
  MappingContext ctx(cloud, map_);

  // Inject transform provider if available
  if (transform_provider_) {
    ctx.setService(transform_provider_);
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