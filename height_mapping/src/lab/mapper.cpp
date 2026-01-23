/*
 * mapper.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/lab/mapper.h"

#include <spdlog/spdlog.h>

#include "height_mapping/api/factory.h"
#include "height_mapping/api/transform_providers.h"
#include "height_mapping/lab/stage_registry.h"

namespace height_mapping::lab {

Mapper::Mapper(const Config& config, IExtrinsicsProvider::Ptr extrinsics,
               IRobotPoseProvider::Ptr pose)
    : extrinsics_(std::move(extrinsics)), pose_(std::move(pose)) {
  if (!extrinsics_) {
    throw std::invalid_argument(
        "[ppl::Mapper] Extrinsics provider is required");
  }
  if (!pose_) {
    throw std::invalid_argument(
        "[ppl::Mapper] Robot pose provider is required");
  }

  map_ = createHeightMap(config.map);
  loadPipeline(config.pipeline);

  // Initialize profiler if enabled
  if (config.enable_profiling) {
    profiler_ = std::make_unique<MappingProfiler>(pipeline_);
    profiler_->setAutoprint(true);
    profiler_->setPrintInterval(config.profile_interval);
    spdlog::info("[ppl::Mapper] Profiling enabled (interval: {})",
                 config.profile_interval);
  }

  spdlog::debug("[ppl::Mapper] Initialized with {}x{}m map, {}m resolution",
                config.map.width, config.map.height, config.map.resolution);
}

Mapper::~Mapper() = default;

void Mapper::loadPipeline(const YAML::Node& yaml_config) {
  if (!yaml_config) {
    spdlog::error("[ppl::Mapper] No pipeline config provided");
    return;
  }

  // Ensure all stages are registered before loading pipeline
  registerAllStages();

  try {
    pipeline_.load(yaml_config);
    spdlog::info("[ppl::Mapper] Pipeline loaded");
  } catch (const std::exception& e) {
    throw std::runtime_error("[ppl::Mapper] Failed to load pipeline: " +
                             std::string(e.what()));
  }
}

void Mapper::update(std::shared_ptr<PointCloud> scan) {
  if (!scan || scan->empty()) {
    spdlog::warn(
        "[ppl::Mapper] Received empty point cloud or null pointer. "
        "Skipping.");
    return;
  }
  if (scan->frameId().empty()) {
    spdlog::error("[ppl::Mapper] Point cloud has no frame ID. Skipping.");
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
      spdlog::warn("[ppl::Mapper] Pipeline returned false");
    }
  } catch (const std::exception& e) {
    spdlog::error("[ppl::Mapper] Pipeline error: {}", e.what());
  }
}

const height_mapping::HeightMap& Mapper::map() const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return *map_;
}

void Mapper::reset() {
  std::lock_guard<std::shared_mutex> lock(map_mutex_);
  if (map_) map_->clearAll();
}

}  // namespace height_mapping::lab
