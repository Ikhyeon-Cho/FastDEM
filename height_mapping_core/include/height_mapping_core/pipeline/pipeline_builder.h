/*
 * pipeline_builder.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_PIPELINE_BUILDER_H
#define HEIGHT_MAPPING_CORE_PIPELINE_PIPELINE_BUILDER_H

#include "height_mapping_core/pipeline/pipeline.h"
#include "height_mapping_core/pipeline/stage_factory.h"
#include "height_mapping_core/pipeline/stages/height_estimation_stage.h"
#include "height_mapping_core/pipeline/stages/map_origin_update_stage.h"
#include "height_mapping_core/pipeline/stages/passthrough_filter_stage.h"
#include "height_mapping_core/pipeline/stages/transform_stage.h"
#include "height_mapping_core/pipeline/stages/voxel_filter_stage.h"
#include "height_mapping_core/config/pipeline_config.h"
#include <optional>

namespace height_mapping::core {

class PipelineBuilder {
public:
  PipelineBuilder() { pipeline_ = std::make_unique<Pipeline>(); }

  // Build pipeline from configuration
  PipelineBuilder& fromConfig(const PipelineConfig& config) {
    config_ = config;
    return *this;
  }

  // Set transform provider for stages that need it
  PipelineBuilder& withTransformProvider(std::shared_ptr<ITransformProvider> provider) {
    transform_provider_ = provider;
    return *this;
  }

  // Transform stage
  PipelineBuilder &withTransform(std::shared_ptr<ITransformProvider> provider,
                                 const std::string &target_frame = "map") {
    pipeline_->addStage(
        std::make_unique<TransformStage>(std::weak_ptr<ITransformProvider>(provider), target_frame));
    return *this;
  }

  PipelineBuilder &withStaticTransform(const Transform3D &transform) {
    pipeline_->addStage(std::make_unique<TransformStage>(transform));
    return *this;
  }

  // Filter stages
  PipelineBuilder &withVoxelFilter(float voxel_size = 0.05f) {
    pipeline_->addStage(std::make_unique<VoxelFilterStage>(voxel_size));
    return *this;
  }

  PipelineBuilder &withPassthroughFilter(float z_min = -100.0f,
                                         float z_max = 100.0f) {
    auto filter = std::make_unique<PassthroughFilterStage>();
    filter->setZLimits(z_min, z_max);
    pipeline_->addStage(std::move(filter));
    return *this;
  }

  PipelineBuilder &withBoxFilter(float x_min, float x_max, float y_min,
                                 float y_max, float z_min, float z_max) {
    auto filter = std::make_unique<PassthroughFilterStage>();
    filter->setXLimits(x_min, x_max);
    filter->setYLimits(y_min, y_max);
    filter->setZLimits(z_min, z_max);
    pipeline_->addStage(std::move(filter));
    return *this;
  }

  PipelineBuilder &withDistanceFilter(float min_distance, float max_distance) {
    auto filter = std::make_unique<PassthroughFilterStage>();
    filter->setDistanceLimits(min_distance, max_distance);
    pipeline_->addStage(std::move(filter));
    return *this;
  }

  // Height estimation
  PipelineBuilder &
  withHeightEstimation(HeightEstimationStage::EstimatorType type =
                           HeightEstimationStage::EstimatorType::KALMAN) {
    pipeline_->addStage(std::make_unique<HeightEstimationStage>(type));
    return *this;
  }

  PipelineBuilder &withKalmanHeightEstimation(float process_noise = 0.01f,
                                              float measurement_noise = 0.1f) {
    auto stage = std::make_unique<HeightEstimationStage>(
        HeightEstimationStage::EstimatorType::KALMAN);
    stage->setKalmanParameters(process_noise, measurement_noise);
    pipeline_->addStage(std::move(stage));
    return *this;
  }

  // Map origin update
  PipelineBuilder &
  withMapOriginUpdate(std::shared_ptr<ITransformProvider> provider,
                      const std::string &robot_frame = "base_link",
                      const std::string &map_frame = "map",
                      MapOriginUpdateStage::UpdateMode mode =
                          MapOriginUpdateStage::UpdateMode::CONTINUOUS,
                      float update_threshold = 1.0f) {
    pipeline_->addStage(std::make_unique<MapOriginUpdateStage>(
        std::weak_ptr<ITransformProvider>(provider), robot_frame, map_frame, mode, update_threshold));
    return *this;
  }

  // Convenience methods for specific update modes
  PipelineBuilder &
  withContinuousMapUpdate(std::shared_ptr<ITransformProvider> provider,
                          const std::string &robot_frame = "base_link",
                          const std::string &map_frame = "map") {
    return withMapOriginUpdate(provider, robot_frame, map_frame,
                               MapOriginUpdateStage::UpdateMode::CONTINUOUS);
  }

  PipelineBuilder &
  withThresholdMapUpdate(std::shared_ptr<ITransformProvider> provider, float threshold = 1.0f,
                         const std::string &robot_frame = "base_link",
                         const std::string &map_frame = "map") {
    return withMapOriginUpdate(provider, robot_frame, map_frame,
                               MapOriginUpdateStage::UpdateMode::THRESHOLD,
                               threshold);
  }

  PipelineBuilder &
  withCenteredMapUpdate(std::shared_ptr<ITransformProvider> provider,
                        const std::string &robot_frame = "base_link",
                        const std::string &map_frame = "map") {
    return withMapOriginUpdate(provider, robot_frame, map_frame,
                               MapOriginUpdateStage::UpdateMode::CENTERED);
  }

  // Custom stage
  PipelineBuilder &withStage(std::unique_ptr<IPipelineStage> stage) {
    pipeline_->addStage(std::move(stage));
    return *this;
  }

  // Configuration
  PipelineBuilder &stopOnError(bool stop = true) {
    pipeline_->setStopOnError(stop);
    return *this;
  }

  PipelineBuilder &withCallback(Pipeline::StageCallback callback) {
    pipeline_->setStageCallback(callback);
    return *this;
  }

  // Build the pipeline
  std::unique_ptr<Pipeline> build() {
    // If config is provided, build from config
    if (config_.has_value()) {
      buildFromConfig();
    }
    return std::move(pipeline_);
  }

  // Preset pipelines
  static std::unique_ptr<Pipeline>
  createDefaultPipeline(std::shared_ptr<ITransformProvider> tf_provider = nullptr) {

    PipelineBuilder builder;

    // Add transform if provider is available
    if (tf_provider) {
      builder.withTransform(tf_provider, "map");
    }

    return builder
        .withPassthroughFilter(-2.0f, 5.0f) // Remove ground and high points
        .withVoxelFilter(0.05f)             // 5cm voxels
        .withHeightEstimation()             // Kalman filter
        .build();
  }

  static std::unique_ptr<Pipeline>
  createFastPipeline(std::shared_ptr<ITransformProvider> tf_provider = nullptr) {

    PipelineBuilder builder;

    if (tf_provider) {
      builder.withTransform(tf_provider, "map");
    }

    return builder
        .withPassthroughFilter(-1.0f, 3.0f) // Aggressive filtering
        .withVoxelFilter(0.1f)              // Larger voxels for speed
        .withHeightEstimation(HeightEstimationStage::EstimatorType::MEAN)
        .build();
  }

  static std::unique_ptr<Pipeline>
  createAccuratePipeline(std::shared_ptr<ITransformProvider> tf_provider = nullptr) {

    PipelineBuilder builder;

    if (tf_provider) {
      builder.withTransform(tf_provider, "map");
    }

    return builder
        .withPassthroughFilter(-5.0f, 10.0f)       // Keep more points
        .withVoxelFilter(0.02f)                    // Fine voxels
        .withKalmanHeightEstimation(0.005f, 0.05f) // Precise Kalman
        .build();
  }

private:
  void buildFromConfig() {
    const auto& cfg = config_.value();
    const auto& stages = cfg.stages;

    // Add stages in the correct order based on config

    // 1. Transform stage (if enabled and provider available)
    if (stages.transform && transform_provider_) {
      withTransform(transform_provider_, cfg.transform.target_frame);
    }

    // 2. Voxel filter
    if (stages.voxel_filter) {
      withVoxelFilter(cfg.filters.voxel_size);
    }

    // 3. Passthrough filter
    if (stages.passthrough_filter) {
      withBoxFilter(cfg.filters.x_min, cfg.filters.x_max,
                   cfg.filters.y_min, cfg.filters.y_max,
                   cfg.filters.z_min, cfg.filters.z_max);
    }

    // 4. Height estimation
    if (stages.height_estimation) {
      HeightEstimationStage::EstimatorType type;
      switch (cfg.estimator.type) {
        case PipelineConfig::Estimator::KALMAN:
          type = HeightEstimationStage::EstimatorType::KALMAN;
          break;
        case PipelineConfig::Estimator::MOVING_AVERAGE:
          type = HeightEstimationStage::EstimatorType::MOVING_AVERAGE;
          break;
        default:
          type = HeightEstimationStage::EstimatorType::INCREMENTAL_MEAN;
          break;
      }
      withHeightEstimation(type);
    }

    // 5. Map origin update
    if (stages.map_update && transform_provider_) {
      MapOriginUpdateStage::UpdateMode mode;
      switch (cfg.map_update.mode) {
        case PipelineConfig::MapUpdate::THRESHOLD:
          mode = MapOriginUpdateStage::UpdateMode::THRESHOLD;
          break;
        case PipelineConfig::MapUpdate::CENTERED:
          mode = MapOriginUpdateStage::UpdateMode::CENTERED;
          break;
        default:
          mode = MapOriginUpdateStage::UpdateMode::CONTINUOUS;
          break;
      }
      withMapOriginUpdate(transform_provider_, cfg.map_update.robot_frame,
                         cfg.transform.target_frame, mode, cfg.map_update.threshold);
    }

    // 6. Configure pipeline behavior
    if (cfg.behavior.stop_on_error) {
      stopOnError(true);
    }
  }

  std::unique_ptr<Pipeline> pipeline_;
  std::optional<PipelineConfig> config_;
  std::shared_ptr<ITransformProvider> transform_provider_;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_PIPELINE_BUILDER_H