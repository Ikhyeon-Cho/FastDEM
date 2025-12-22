/*
 * height_mapper.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/ppl/height_mapper.h"

#include <spdlog/spdlog.h>

#include "height_mapping/ppl/stage_registry.h"

namespace height_mapping::ppl {

HeightMapper::HeightMapper(const Config& config,
                           IExtrinsicsProvider::Ptr extrinsics,
                           IRobotPoseProvider::Ptr pose)
    : config_(config),
      extrinsics_(std::move(extrinsics)),
      pose_provider_(std::move(pose)) {
  if (!extrinsics_) {
    throw std::invalid_argument(
        "[ppl::HeightMapper] Extrinsics provider is required");
  }
  if (!pose_provider_) {
    throw std::invalid_argument(
        "[ppl::HeightMapper] Robot pose provider is required");
  }

  // Initialize map
  map_ = std::make_shared<height_mapping::HeightMap>(config_.map);

  // Ensure all stages are registered before loading pipeline
  registerAllStages();

  // Load pipeline from YAML
  if (!config_.pipeline_config_path.empty()) {
    try {
      pipeline_.load(config_.pipeline_config_path);
      spdlog::info("[ppl::HeightMapper] Pipeline loaded from: {}",
                   config_.pipeline_config_path);
    } catch (const std::exception& e) {
      throw std::runtime_error("[ppl::HeightMapper] Failed to load pipeline: " +
                               std::string(e.what()));
    }
  } else {
    spdlog::warn("[ppl::HeightMapper] No pipeline config path specified");
  }

  spdlog::debug(
      "[ppl::HeightMapper] Initialized with {}x{}m map, {}m resolution",
      config_.map.width, config_.map.height, config_.map.resolution);
}

void HeightMapper::integrate(std::shared_ptr<PointCloud> scan) {
  if (!scan || scan->empty()) {
    spdlog::warn(
        "[ppl::HeightMapper] Received empty point cloud or null pointer. "
        "Skipping.");
    return;
  }
  if (scan->frameId().empty()) {
    spdlog::error("[ppl::HeightMapper] Point cloud has no frame ID. Skipping.");
    return;
  }

  // Lookup extrinsic (sensor -> base)
  auto T_base_sensor_opt = extrinsics_->getExtrinsic(scan->frameId());
  if (!T_base_sensor_opt) {
    spdlog::warn("Extrinsic not available for sensor '{}'. Skipping.",
                 scan->frameId());
    return;
  }

  // Lookup robot pose (base -> map)
  auto T_map_base_opt = pose_provider_->getRobotPoseAt(scan->timestamp());
  if (!T_map_base_opt) {
    spdlog::warn("Robot pose not available at timestamp {}. Skipping.",
                 scan->timestamp());
    return;
  }

  // Unwrap optionals (safe after early return checks)
  const auto& T_base_sensor = *T_base_sensor_opt;
  const auto& T_map_base = *T_map_base_opt;

  std::lock_guard<std::shared_mutex> lock(map_mutex_);

  // Create frame with SHARED ownership of the input cloud (Zero-Copy)
  auto frame = std::make_shared<MappingFrame>(scan, map_);
  frame->pose = T_map_base;
  frame->extrinsic = T_base_sensor;

  // Execute mapping pipeline
  try {
    if (!pipeline_.run(frame)) {
      spdlog::warn("[ppl::HeightMapper] Pipeline returned false");
    }
  } catch (const std::exception& e) {
    spdlog::error("[ppl::HeightMapper] Pipeline error: {}", e.what());
  }
}

const height_mapping::HeightMap& HeightMapper::getHeightMap() const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return *map_;
}

void HeightMapper::reset() {
  std::lock_guard<std::shared_mutex> lock(map_mutex_);
  if (map_) map_->clear();
}

}  // namespace height_mapping::ppl
