/*
 * mapper.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/api/mapper.h"

#include <spdlog/spdlog.h>

#include <nanopcl/filters/crop.hpp>
#include <nanopcl/filters/downsample.hpp>
#include <nanopcl/transform/transform_ops.hpp>

#include "height_mapping/ops/inpainting.h"
#include "height_mapping/ops/raycasting.h"

namespace height_mapping {

Mapper::Mapper(const Config& config) : config_(config) {
  map_ = std::make_unique<HeightMap>();
  map_->setGeometry(config_.map.width, config_.map.height,
                    config_.map.resolution);
  map_->setFrameId(config_.map.frame_id);
  initializeHeightEstimator();

  spdlog::debug("[Mapper] Initialized with {}x{}m map, {}m resolution",
                config_.map.width, config_.map.height, config_.map.resolution);
}

Mapper::~Mapper() = default;

void Mapper::initializeHeightEstimator() {
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

void Mapper::integrate(const PointCloud& cloud_in_sensor,
                       const Transformf& T_base_sensor,
                       const Transformf& T_map_base) {
  if (cloud_in_sensor.empty()) {
    return;
  }

  // Lock the map for writing
  std::lock_guard<std::shared_mutex> lock(map_mutex_);

  // 0. Robot-centric mode: Move map center to robot
  if (config_.map.mode == config::MappingMode::LOCAL) {
    grid_map::Position robot_pos(T_map_base.translation().x(),
                                 T_map_base.translation().y());
    map_->move(robot_pos);
  }

  // 1. Voxel grid first (reduces points before transform)
  auto scan =
      npcl::filters::voxelGrid(cloud_in_sensor, config_.voxel_filter.voxel_size,
                               config_.voxel_filter.method);

  // 2. Transform to base frame, then apply spatial filters (meaningful in base)
  scan = npcl::transformCloud(std::move(scan), T_base_sensor);
  scan = npcl::filters::cropZ(std::move(scan), config_.spatial_filter.z_min,
                              config_.spatial_filter.z_max);
  scan = npcl::filters::cropRange(std::move(scan),
                                  config_.spatial_filter.range_min,
                                  config_.spatial_filter.range_max);

  // 3. Transform to map frame
  scan = npcl::transformCloud(std::move(scan), T_map_base);

  // 4. Raycasting (ghost removal + persistence management)
  if (config_.raycasting.enabled) {
    auto T_map_sensor = T_map_base * T_base_sensor;
    Eigen::Vector3f sensor_origin = T_map_sensor.translation();
    ops::applyRaycasting(*map_, scan, sensor_origin, config_.raycasting);
  }

  // 5. Update height map
  updater::Elevation elevation(*height_estimator_);
  height_mapping::update(*map_, scan, elevation);

  // 6. Inpainting (fill NaN holes in separate layer)
  ops::applyInpainting(*map_, config_.inpainting);
}

void Mapper::moveOrigin(const Eigen::Vector2f& position) {
  std::lock_guard<std::shared_mutex> lock(map_mutex_);
  map_->move(grid_map::Position(position.x(), position.y()));
}

const HeightMap& Mapper::map() const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return *map_;
}

HeightMap& Mapper::map() {
  std::lock_guard<std::shared_mutex> lock(map_mutex_);
  return *map_;
}

void Mapper::reset() {
  std::lock_guard<std::shared_mutex> lock(map_mutex_);
  if (map_) {
    map_->clear();
  }
}

}  // namespace height_mapping
