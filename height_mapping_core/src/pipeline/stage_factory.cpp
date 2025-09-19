/*
 * stage_factory.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/pipeline/stage_factory.h"
#include "height_mapping_core/pipeline/stages/height_estimation_stage.h"
#include "height_mapping_core/pipeline/stages/map_origin_update_stage.h"
#include "height_mapping_core/pipeline/stages/passthrough_filter_stage.h"
#include "height_mapping_core/pipeline/stages/transform_stage.h"
#include "height_mapping_core/pipeline/stages/voxel_filter_stage.h"
#include <stdexcept>

namespace height_mapping::core {

// Static registry for stage creators
std::unordered_map<std::string, StageFactory::StageCreator>&
StageFactory::getRegistry() {
  static std::unordered_map<std::string, StageCreator> registry;
  return registry;
}

void StageFactory::registerStage(const std::string& type, StageCreator creator) {
  getRegistry()[type] = creator;
}

std::unique_ptr<IPipelineStage> StageFactory::create(
    const StageConfig& config,
    std::shared_ptr<ITransformProvider> transform_provider) {

  auto& registry = getRegistry();

  // Initialize default creators if registry is empty
  if (registry.empty()) {
    // Register transform stage
    registerStage("transform", [](const StageConfig& cfg, std::shared_ptr<ITransformProvider> tf) {
      if (!tf) {
        throw std::invalid_argument("Transform stage requires transform provider");
      }
      auto target_frame = cfg.getParam<std::string>("target_frame", "map");
      // Pass weak_ptr to TransformStage
      return std::make_unique<TransformStage>(std::weak_ptr<ITransformProvider>(tf), target_frame);
    });

    // Register voxel filter stage
    registerStage("voxel_filter", [](const StageConfig& cfg, std::shared_ptr<ITransformProvider>) {
      auto voxel_size = cfg.getParam<double>("voxel_size", 0.1);
      return std::make_unique<VoxelFilterStage>(voxel_size);
    });

    // Register passthrough filter stage
    registerStage("passthrough_filter", [](const StageConfig& cfg, std::shared_ptr<ITransformProvider>) {
      auto stage = std::make_unique<PassthroughFilterStage>();

      // Configure bounds
      auto x_min = cfg.getParam<double>("x_min", -std::numeric_limits<double>::max());
      auto x_max = cfg.getParam<double>("x_max", std::numeric_limits<double>::max());
      auto y_min = cfg.getParam<double>("y_min", -std::numeric_limits<double>::max());
      auto y_max = cfg.getParam<double>("y_max", std::numeric_limits<double>::max());
      auto z_min = cfg.getParam<double>("z_min", -std::numeric_limits<double>::max());
      auto z_max = cfg.getParam<double>("z_max", std::numeric_limits<double>::max());

      stage->setXLimits(x_min, x_max);
      stage->setYLimits(y_min, y_max);
      stage->setZLimits(z_min, z_max);

      return stage;
    });

    // Register height estimation stage
    registerStage("height_estimation", [](const StageConfig& cfg, std::shared_ptr<ITransformProvider>) {
      auto estimator_type = cfg.getParam<std::string>("estimator_type", "incremental_mean");

      HeightEstimationStage::EstimatorType type;
      if (estimator_type == "kalman") {
        type = HeightEstimationStage::EstimatorType::KALMAN;
      } else if (estimator_type == "moving_average") {
        type = HeightEstimationStage::EstimatorType::MOVING_AVERAGE;
      } else {
        type = HeightEstimationStage::EstimatorType::INCREMENTAL_MEAN;
      }

      return std::make_unique<HeightEstimationStage>(type);
    });

    // Register map origin update stage
    registerStage("map_update", [](const StageConfig& cfg, std::shared_ptr<ITransformProvider> tf) {
      if (!tf) {
        throw std::invalid_argument("Map update stage requires transform provider");
      }

      auto robot_frame = cfg.getParam<std::string>("robot_frame", "base_link");
      auto map_frame = cfg.getParam<std::string>("map_frame", "map");
      auto mode_str = cfg.getParam<std::string>("mode", "continuous");
      auto threshold = cfg.getParam<double>("threshold", 5.0);

      MapOriginUpdateStage::UpdateMode mode;
      if (mode_str == "threshold") {
        mode = MapOriginUpdateStage::UpdateMode::THRESHOLD;
      } else if (mode_str == "centered") {
        mode = MapOriginUpdateStage::UpdateMode::CENTERED;
      } else {
        mode = MapOriginUpdateStage::UpdateMode::CONTINUOUS;
      }

      // Pass weak_ptr to MapOriginUpdateStage
      return std::make_unique<MapOriginUpdateStage>(std::weak_ptr<ITransformProvider>(tf), robot_frame, map_frame, mode, threshold);
    });
  }

  // Find and execute creator
  auto it = registry.find(config.type);
  if (it == registry.end()) {
    throw std::runtime_error("Unknown stage type: " + config.type);
  }

  return it->second(config, transform_provider);
}

// Note: createFromPipelineConfig has been removed
// PipelineBuilder now handles config parsing and uses StageFactory internally
// This keeps StageFactory focused on its single responsibility: creating individual stages

} // namespace height_mapping::core