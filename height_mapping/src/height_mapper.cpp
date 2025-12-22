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

#include "height_mapping/estimators/incremental_mean.h"
#include "height_mapping/estimators/kalman_filter.h"
#include "height_mapping/estimators/moving_average.h"

namespace height_mapping {

HeightMapper::HeightMapper(const Config& config,
                           IExtrinsicsProvider::Ptr extrinsics,
                           IRobotPoseProvider::Ptr pose)
    : config_(config),
      extrinsics_(std::move(extrinsics)),
      pose_(std::move(pose)) {
  if (!extrinsics_) {
    throw std::invalid_argument("Extrinsics provider is required");
  }
  if (!pose_) {
    throw std::invalid_argument("Robot pose provider is required");
  }

  map_ = std::make_unique<HeightMap>(config_.map);
  initializeAlgorithms();

  spdlog::debug("HeightMapper initialized: {}x{}m map, {}m resolution",
                config_.map.width, config_.map.height, config_.map.resolution);
}

void HeightMapper::initializeAlgorithms() {
  // Stateful filters need config set once
  voxel_filter_.setConfig(config_.voxel_filter);
  passthrough_filter_.setConfig(config_.passthrough_filter);

  // Height estimator
  const auto& est = config_.height_estimation;
  if (est.type == "kalman_filter") {
    height_estimator_ = std::make_unique<estimators::KalmanFilter>(est.kalman);
  } else if (est.type == "moving_average") {
    height_estimator_ =
        std::make_unique<estimators::MovingAverage>(est.moving_average);
  } else {
    height_estimator_ = std::make_unique<estimators::IncrementalMean>();
  }
}

void HeightMapper::integrate(std::shared_ptr<PointCloud> scan) {
  if (!scan || scan->empty()) {
    spdlog::warn("[HeightMapper] Received empty points or null pointer. Skipping "
                 "integration.");
    return;
  }
  if (scan->frameId().empty()) {
    spdlog::error("[HeightMapper] Input scan has no frameId. Can't transform.");
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

  // Lock the map for writing
  std::lock_guard<std::shared_mutex> lock(map_mutex_);

  // 1. Move map origin to robot position (robot-centric mode)
  if (config_.mode == MappingMode::ROBOT_CENTRIC) {
    grid_map::Position robot_pos(T_map_base.translation().x(),
                                 T_map_base.translation().y());
    map_->move(robot_pos);
  }

  // 2. Transform: sensor -> base
  transformInPlace(*scan, T_base_sensor);

  // 3. Voxel filter
  voxel_filter_.filterInPlace(*scan);

  // 4. Passthrough filter
  passthrough_filter_.filterInPlace(*scan);

  // 5. Ground segmentation (in base frame)
  if (config_.ground_segmentation.method == "statistical") {
    algorithms::segmentGroundStatistical(
        *scan, config_.ground_segmentation.statistical);
  } else if (config_.ground_segmentation.method == "grid") {
    algorithms::segmentGroundGrid(*scan, config_.ground_segmentation.grid);
  }
  // else: method == "none" or empty -> skip

  // 6. Transform: base -> map (auto frame check + frameId update)
  transformInPlace(*scan, T_map_base);

  // 7. Raycasting
  if (config_.raycasting.enable_correction) {
    auto T_map_sensor = T_map_base * T_base_sensor;
    Eigen::Vector3f sensor_origin = T_map_sensor.translation();
    algorithms::applyRaycasting(*map_, *scan, sensor_origin,
                                config_.raycasting);
  }

  // 8. Height estimation
  algorithms::updateHeightMap(*map_, *scan, *height_estimator_);
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
