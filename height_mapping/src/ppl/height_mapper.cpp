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

#include "height_mapping/interfaces/extrinsics_provider.h"
#include "height_mapping/interfaces/robot_pose_provider.h"
#include "height_mapping/ppl/stage_registry.h"

namespace height_mapping::ppl {

HeightMapper::HeightMapper(const Config& config,
                           IExtrinsicsProvider::Ptr extrinsics,
                           IRobotPoseProvider::Ptr pose)
    : extrinsics_(std::move(extrinsics)), pose_(std::move(pose)) {
  if (!extrinsics_) {
    throw std::invalid_argument(
        "[ppl::HeightMapper] Extrinsics provider is required");
  }
  if (!pose_) {
    throw std::invalid_argument(
        "[ppl::HeightMapper] Robot pose provider is required");
  }

  map_ = std::make_shared<HeightMap>(config.map);
  loadPipeline(config.pipeline);

  // Initialize profiler if enabled
  if (config.enable_profiling) {
    profiler_ = std::make_unique<MappingProfiler>(pipeline_);
    profiler_->setAutoprint(true);
    profiler_->setPrintInterval(config.profile_interval);
    spdlog::info("[ppl::HeightMapper] Profiling enabled (interval: {})",
                 config.profile_interval);
  }

  spdlog::debug(
      "[ppl::HeightMapper] Initialized with {}x{}m map, {}m resolution",
      config.map.width, config.map.height, config.map.resolution);
}

HeightMapper::~HeightMapper() = default;

void HeightMapper::loadPipeline(const YAML::Node& yaml_config) {
  if (!yaml_config) {
    spdlog::error("[ppl::HeightMapper] No pipeline config provided");
    return;
  }

  // Ensure all stages are registered before loading pipeline
  registerAllStages();

  try {
    pipeline_.load(yaml_config);
    spdlog::info("[ppl::HeightMapper] Pipeline loaded");
  } catch (const std::exception& e) {
    throw std::runtime_error("[ppl::HeightMapper] Failed to load pipeline: " +
                             std::string(e.what()));
  }
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
  auto T_base_sensor_opt = extrinsics_->getExtrinsic(scan->frameId());
  if (!T_base_sensor_opt) {
    spdlog::warn("Extrinsic not available for sensor '{}'. Skipping.",
                 scan->frameId());
    return;
  }
  auto T_map_base_opt = pose_->getRobotPoseAt(scan->timestamp());
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
  frame->robot_pose = T_map_base;
  frame->extrinsic = T_base_sensor;

  // Execute mapping pipeline
  try {
    bool success = profiler_ ? profiler_->run(frame) : pipeline_.run(frame);
    if (!success) {
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
