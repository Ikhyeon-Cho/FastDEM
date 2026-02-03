// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * fastdem.cpp
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/fastdem.hpp"

#include <spdlog/spdlog.h>

#include <nanopcl/filters/crop.hpp>

#include "fastdem/postprocess/inpainting.hpp"
#include "fastdem/postprocess/raycasting.hpp"
#include "fastdem/postprocess/spatial_smoothing.hpp"
#include "fastdem/postprocess/uncertainty_fusion.hpp"

namespace fastdem {

FastDEM::FastDEM(ElevationMap& map) : FastDEM(map, Config{}) {}

FastDEM::FastDEM(ElevationMap& map, const Config& cfg) : map_(map), cfg_(cfg) {
  sensor_model_ = createSensorModel(cfg_.sensor);
  rasterization_ = createRasterization(cfg_.rasterization);
  mapping_ = createElevationMapping(map_, cfg_.mapping);
}

FastDEM::~FastDEM() = default;

FastDEM& FastDEM::setMappingMode(MappingMode mode) {
  cfg_.mapping.mode = mode;
  mapping_ = createElevationMapping(map_, cfg_.mapping);
  return *this;
}

FastDEM& FastDEM::setEstimatorType(EstimationType type) {
  cfg_.mapping.estimation_type = type;
  mapping_ = createElevationMapping(map_, cfg_.mapping);
  return *this;
}

FastDEM& FastDEM::setSensorModel(SensorType type) {
  cfg_.sensor.type = type;
  sensor_model_ = createSensorModel(cfg_.sensor);
  return *this;
}

FastDEM& FastDEM::setHeightRange(float z_min, float z_max) noexcept {
  cfg_.scan_filter.z_min = z_min;
  cfg_.scan_filter.z_max = z_max;
  return *this;
}

FastDEM& FastDEM::setDistanceRange(float range_min, float range_max) noexcept {
  cfg_.scan_filter.range_min = range_min;
  cfg_.scan_filter.range_max = range_max;
  return *this;
}

FastDEM& FastDEM::enableRaycasting(bool enabled) noexcept {
  cfg_.raycasting.enabled = enabled;
  return *this;
}

FastDEM& FastDEM::enableInpainting(bool enabled) noexcept {
  cfg_.inpainting.enabled = enabled;
  return *this;
}

FastDEM& FastDEM::enableUncertaintyFusion(bool enabled) noexcept {
  cfg_.uncertainty_fusion.enabled = enabled;
  return *this;
}

FastDEM& FastDEM::setCalibrationSystem(
    std::shared_ptr<Calibration> calibration) {
  std::unique_lock lock(map_mutex_);
  calibration_ = std::move(calibration);
  return *this;
}

FastDEM& FastDEM::setOdometrySystem(std::shared_ptr<Odometry> odometry) {
  std::unique_lock lock(map_mutex_);
  odometry_ = std::move(odometry);
  return *this;
}

bool FastDEM::hasTransformSystems() const noexcept {
  std::shared_lock lock(map_mutex_);
  return calibration_ != nullptr && odometry_ != nullptr;
}

bool FastDEM::integrate(std::shared_ptr<PointCloud> cloud) {
  // Online-specific validation
  if (!calibration_ || !odometry_) {
    spdlog::error(
        "[FastDEM] Online mode requires transform systems. "
        "Call setTransformSystem() or "
        "setCalibrationSystem()/setOdometrySystem() first, "
        "or use integrate(cloud, T_base_sensor, T_world_base) for offline "
        "mode.");
    return false;
  }

  if (!cloud || cloud->empty()) {
    spdlog::warn("[FastDEM] Received empty or null cloud. Skipping...");
    return false;
  }

  if (cloud->frameId().empty()) {
    spdlog::error("[FastDEM] Input cloud has no frameId. Skipping...");
    return false;
  }

  auto T_base_sensor = calibration_->getExtrinsic(cloud->frameId());
  if (!T_base_sensor) {
    spdlog::warn("[FastDEM] Calibration not available for '{}'. Skipping...",
                 cloud->frameId());
    return false;
  }

  auto T_world_base = odometry_->getPoseAt(cloud->timestamp());
  if (!T_world_base) {
    spdlog::warn("[FastDEM] Odometry not available at {}. Skipping...",
                 cloud->timestamp());
    return false;
  }

  return integrateImpl(*cloud, *T_base_sensor, *T_world_base);
}

bool FastDEM::integrate(const PointCloud& cloud,
                        const Eigen::Isometry3d& T_base_sensor,
                        const Eigen::Isometry3d& T_world_base) {
  // Offline-specific validation
  if (cloud.empty()) return false;

  return integrateImpl(cloud, T_base_sensor, T_world_base);
}

bool FastDEM::integrateImpl(const PointCloud& cloud,
                            const Eigen::Isometry3d& T_base_sensor,
                            const Eigen::Isometry3d& T_world_base) {
  // 1. Compute sensor covariances (in sensor frame, before filtering)
  PointCloud points = cloud;
  sensor_model_->computeSensorCovariances(points);

  // 2. Preprocess scan (transform + filter, covariances preserved)
  const auto& z_min = cfg_.scan_filter.z_min;
  const auto& z_max = cfg_.scan_filter.z_max;
  const auto& range_min = cfg_.scan_filter.range_min;
  const auto& range_max = cfg_.scan_filter.range_max;

  points = nanopcl::transformCloud(std::move(points), T_base_sensor);
  points = nanopcl::filters::cropZ(std::move(points), z_min, z_max);
  points = nanopcl::filters::cropRange(std::move(points), range_min, range_max);
  points = nanopcl::transformCloud(std::move(points), T_world_base,
                                   map_.getFrameId());
  if (points.empty()) return false;
  if (processed_cloud_callback_) {
    processed_cloud_callback_(points);
  }

  // 3. Transform covariances to map frame
  if (points.hasCovariance()) {
    const Eigen::Matrix3f R =
        (T_world_base * T_base_sensor).rotation().cast<float>();
    for (size_t i : points.indices()) {
      auto& cov = points.covariance(i);
      cov = R * cov * R.transpose();
    }
  }

  // 4. Rasterize
  PointCloud rasterized_points = rasterization_->process(points, map_);
  if (rasterized_points.empty()) return false;

  // 5. Map update (thread-safe)
  {
    std::lock_guard<std::shared_mutex> lock(map_mutex_);

    const Eigen::Vector2d robot_position = T_world_base.translation().head<2>();
    mapping_->update(rasterized_points, robot_position);

    // Post-processing: smooth elevation_max → elevation_max_smoothed
    if (map_.exists(layer::elevation_max)) {
      if (!map_.exists(layer::elevation_max_smoothed))
        map_.add(layer::elevation_max_smoothed, NAN);
      map_.get(layer::elevation_max_smoothed) = map_.get(layer::elevation_max);
      applySpatialSmoothing(map_, layer::elevation_max_smoothed);
    }

    const Eigen::Vector3f sensor_origin =
        (T_world_base * T_base_sensor).translation().cast<float>();
    if (cfg_.raycasting.enabled)
      applyRaycasting(map_, points, sensor_origin, cfg_.raycasting);
    if (cfg_.inpainting.enabled) applyInpainting(map_, cfg_.inpainting);
    if (cfg_.uncertainty_fusion.enabled)
      applyUncertaintyFusion(map_, cfg_.uncertainty_fusion);
  }
  return true;
}

void FastDEM::setProcessedCloudCallback(ProcessedCloudCallback callback) {
  processed_cloud_callback_ = std::move(callback);
}

// ─── rasterize ──────────────────────────────────────────────────────────

void rasterize(const PointCloud& cloud, ElevationMap& map,
                   RasterMethod method) {
  if (cloud.empty()) return;

  Rasterization raster(method);
  PointCloud rasterized = raster.process(cloud, map);

  const bool has_intensity =
      rasterized.hasIntensity() && map.exists(layer::intensity);

  for (size_t i : rasterized.indices()) {
    auto pt = rasterized.point(i);
    grid_map::Index index;
    if (!map.getIndex(grid_map::Position(pt.x(), pt.y()), index)) continue;

    map.at(layer::elevation, index) = pt.z();

    if (has_intensity) {
      map.at(layer::intensity, index) = rasterized.intensity(i);
    }
  }
}

ElevationMap rasterize(const PointCloud& cloud, float resolution,
                           RasterMethod method) {
  if (cloud.empty()) return {};

  // Compute XY bounding box
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();

  for (size_t i : cloud.indices()) {
    auto pt = cloud.point(i);
    min_x = std::min(min_x, pt.x());
    min_y = std::min(min_y, pt.y());
    max_x = std::max(max_x, pt.x());
    max_y = std::max(max_y, pt.y());
  }

  // Add one cell margin so edge points are inside the map
  float width = max_x - min_x + resolution;
  float height = max_y - min_y + resolution;

  ElevationMap map;
  map.setGeometry(width, height, resolution);
  map.setPosition(
      grid_map::Position((min_x + max_x) / 2.0, (min_y + max_y) / 2.0));

  rasterize(cloud, map, method);
  return map;
}

}  // namespace fastdem
