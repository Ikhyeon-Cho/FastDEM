/*
 * height_mapper.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/height_mapper.h"

#include <spdlog/spdlog.h>

#include <nanopcl/filters/crop.hpp>
#include <nanopcl/filters/downsample.hpp>
#include <nanopcl/transform/transform_ops.hpp>

#include "height_mapping/algorithms/algorithms.h"
#include "height_mapping/estimators/incremental_mean.h"
#include "height_mapping/height_mapper_config.h"
#include "height_mapping/interfaces/extrinsics_provider.h"
#include "height_mapping/interfaces/robot_pose_provider.h"

namespace height_mapping {

HeightMapper::HeightMapper(const Config& config,
                           std::shared_ptr<IExtrinsicsProvider> extrinsics,
                           std::shared_ptr<IRobotPoseProvider> pose)
    : extrinsics_(std::move(extrinsics)), pose_(std::move(pose)) {
  if (!extrinsics_) {
    throw std::invalid_argument("Extrinsics provider is required");
  }
  if (!pose_) {
    throw std::invalid_argument("Robot pose provider is required");
  }

  config_ = std::make_unique<Config>(config);
  map_ = std::make_unique<HeightMap>(config_->map);
  initializeHeightEstimator();

  spdlog::debug("[HeightMapper] Initialized with {}x{}m map, {}m resolution",
                config_->map.width, config_->map.height,
                config_->map.resolution);
}

HeightMapper::~HeightMapper() = default;

void HeightMapper::initializeHeightEstimator() {
  const auto& params = config_->height_estimation;
  if (params.type == "kalman_filter") {
    height_estimator_ =
        std::make_unique<estimators::KalmanFilter>(params.kalman);
  } else if (params.type == "moving_average") {
    height_estimator_ =
        std::make_unique<estimators::MovingAverage>(params.moving_average);
  } else {
    height_estimator_ = std::make_unique<estimators::IncrementalMean>();
  }
}

void HeightMapper::integrate(std::shared_ptr<PointCloud> scan) {
  if (!scan || scan->empty()) {
    spdlog::warn(
        "[HeightMapper] Received empty points or null pointer. Skipping "
        "integration.");
    return;
  }
  if (scan->frameId().empty()) {
    spdlog::error("[HeightMapper] Input scan has no frameId. Can't transform.");
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
  if (config_->mode == MappingMode::ROBOT_CENTRIC) {
    grid_map::Position robot_pos(T_map_base.translation().x(),
                                 T_map_base.translation().y());
    map_->move(robot_pos);
  }

  // 1. Scan pre-processing: transform to base frame, voxel, spatial filter
  *scan = npcl::transformCloud(std::move(*scan), T_base_sensor);
  *scan = npcl::filters::voxelGrid(std::move(*scan),
                                   config_->voxel_filter.voxel_size,
                                   config_->voxel_filter.method);
  *scan = npcl::filters::cropZ(std::move(*scan), config_->spatial_filter.z_min,
                               config_->spatial_filter.z_max);
  *scan = npcl::filters::cropRange(std::move(*scan),
                                   config_->spatial_filter.range_min,
                                   config_->spatial_filter.range_max);

  // 2. Transform to map frame
  *scan = npcl::transformCloud(std::move(*scan), T_map_base);

  // 3. Raycasting (ghost removal + persistence management)
  if (config_->raycasting.enable_correction) {
    auto T_map_sensor = T_map_base * T_base_sensor;
    Eigen::Vector3f sensor_origin = T_map_sensor.translation();
    algorithms::applyRaycasting(*map_, *scan, sensor_origin,
                                config_->raycasting);
  }

  // 4. Update height map
  algorithms::updateHeightMap(*map_, *scan, *height_estimator_);

  // 5. Inpainting (fill NaN holes in separate layer)
  algorithms::applyInpainting(*map_, config_->inpainting);
}

const HeightMap& HeightMapper::getHeightMap() const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return *map_;
}

void HeightMapper::reset() {
  std::lock_guard<std::shared_mutex> lock(map_mutex_);
  if (map_) {
    map_->clear();
  }
}

}  // namespace height_mapping
