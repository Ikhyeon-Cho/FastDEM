/*
 * online_mapper.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/api/online_mapper.h"

#include <spdlog/spdlog.h>

#include <nanopcl/filters/crop.hpp>
#include <nanopcl/filters/downsample.hpp>
#include <nanopcl/transform/transform_ops.hpp>

#include "height_mapping/api/config.h"
#include "height_mapping/api/transform_providers.h"
#include "height_mapping/ops/inpainting.h"
#include "height_mapping/ops/raycasting.h"

namespace height_mapping {

OnlineMapper::OnlineMapper(const Config& config,
                           std::shared_ptr<IExtrinsicsProvider> extrinsics,
                           std::shared_ptr<IRobotPoseProvider> pose)
    : config_(config),
      extrinsics_(std::move(extrinsics)),
      pose_(std::move(pose)) {
  if (!extrinsics_) {
    throw std::invalid_argument("Extrinsics provider is required");
  }
  if (!pose_) {
    throw std::invalid_argument("Robot pose provider is required");
  }

  map_ = std::make_unique<HeightMap>();
  map_->setGeometry(config_.map.width,   //
                    config_.map.height,  //
                    config_.map.resolution);
  map_->setFrameId(config_.map.frame_id);
  initializeHeightEstimator();

  spdlog::debug("[OnlineMapper] Initialized with {}x{}m map, {}m resolution",
                config_.map.width, config_.map.height, config_.map.resolution);
}

OnlineMapper::~OnlineMapper() = default;

void OnlineMapper::initializeHeightEstimator() {
  if (config_.estimation.type == "kalman_filter") {
    height_estimator_ = std::make_unique<estimator::KalmanFilter>(
        config_.estimation.kalman.process_noise,
        config_.estimation.kalman.measurement_noise,
        config_.estimation.kalman.initial_variance);
  } else if (config_.estimation.type == "moving_average") {
    height_estimator_ = std::make_unique<estimator::MovingAverage>(
        config_.estimation.moving_average.alpha);
  } else {
    height_estimator_ = std::make_unique<estimator::IncrementalMean>();
  }
}

void OnlineMapper::integrate(std::shared_ptr<PointCloud> scan) {
  if (!scan || scan->empty()) {
    spdlog::warn(
        "[OnlineMapper] Received empty points or null pointer. Skipping "
        "integration.");
    return;
  }
  if (scan->frameId().empty()) {
    spdlog::error("[OnlineMapper] Input scan has no frameId. Can't transform.");
    return;
  }
  // For multi-sensor scenario, we get the extrinsics for each sensor
  auto sensor_extrinsic_opt = extrinsics_->getExtrinsic(scan->frameId());
  if (!sensor_extrinsic_opt) {
    spdlog::warn("Extrinsic not available for sensor '{}'. Skipping.",
                 scan->frameId());
    return;
  }
  auto robot_pose_opt = pose_->getRobotPoseAt(scan->timestamp());
  if (!robot_pose_opt) {
    spdlog::warn("Robot pose not available at timestamp {}. Skipping.",
                 scan->timestamp());
    return;
  }

  // Unwrap optionals (safe after early return checks)
  const auto& T_base_sensor = *sensor_extrinsic_opt;
  const auto& T_map_base = *robot_pose_opt;

  // Lock the map for writing
  std::lock_guard<std::shared_mutex> lock(map_mutex_);

  // 0. Robot-centric mode: Move map center to robot
  if (config_.map.mode == config::MappingMode::LOCAL) {
    grid_map::Position robot_pos(T_map_base.translation().x(),
                                 T_map_base.translation().y());
    map_->move(robot_pos);
  }

  // 1. Scan pre-processing: voxel first (reduces points before transform)
  *scan = npcl::filters::voxelGrid(std::move(*scan),
                                   config_.voxel_filter.voxel_size,
                                   config_.voxel_filter.method);

  // 2. Transform to base frame, then apply spatial filters (meaningful in base)
  *scan = npcl::transformCloud(std::move(*scan), T_base_sensor);
  *scan = npcl::filters::cropZ(std::move(*scan), config_.spatial_filter.z_min,
                               config_.spatial_filter.z_max);
  *scan = npcl::filters::cropRange(std::move(*scan),
                                   config_.spatial_filter.range_min,
                                   config_.spatial_filter.range_max);

  // 3. Transform to map frame
  *scan = npcl::transformCloud(std::move(*scan), T_map_base);

  // 4. Raycasting (ghost removal + persistence management)
  if (config_.raycasting.enabled) {
    auto T_map_sensor = T_map_base * T_base_sensor;
    Eigen::Vector3f sensor_origin = T_map_sensor.translation();
    ops::applyRaycasting(*map_, *scan, sensor_origin, config_.raycasting);
  }

  // 5. Update height map
  updater::Elevation elevation(*height_estimator_);
  height_mapping::update(*map_, *scan, elevation);

  // 6. Inpainting (fill NaN holes in separate layer)
  ops::applyInpainting(*map_, config_.inpainting);
}

const HeightMap& OnlineMapper::map() const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return *map_;
}

void OnlineMapper::reset() {
  std::lock_guard<std::shared_mutex> lock(map_mutex_);
  if (map_) {
    map_->clear();
  }
}

}  // namespace height_mapping
