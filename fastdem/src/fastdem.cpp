// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include "fastdem/fastdem.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <limits>
#include <nanopcl/filters/crop.hpp>
#include <nanopcl/filters/downsample.hpp>

#include "fastdem/mapping/grid_index_hash.hpp"
#include "fastdem/postprocess/raycasting.hpp"

namespace fastdem {

FastDEM::FastDEM(ElevationMap& map) : FastDEM(map, Config{}) {}

FastDEM::FastDEM(ElevationMap& map, const Config& cfg) : map_(map), cfg_(cfg) {
  sensor_model_ = createSensorModel(cfg_.sensor_model);
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
  cfg_.sensor_model.type = type;
  sensor_model_ = createSensorModel(cfg_.sensor_model);
  return *this;
}

FastDEM& FastDEM::setSensorModel(std::unique_ptr<SensorModel> model) noexcept {
  sensor_model_ = std::move(model);
  return *this;
}

FastDEM& FastDEM::setHeightFilter(float z_min, float z_max) noexcept {
  cfg_.point_filter.z_min = z_min;
  cfg_.point_filter.z_max = z_max;
  return *this;
}

FastDEM& FastDEM::setRangeFilter(float range_min, float range_max) noexcept {
  cfg_.point_filter.range_min = range_min;
  cfg_.point_filter.range_max = range_max;
  return *this;
}

FastDEM& FastDEM::enableRaycasting(bool enabled) noexcept {
  cfg_.raycasting.enabled = enabled;
  return *this;
}

FastDEM& FastDEM::setCalibrationProvider(
    std::shared_ptr<Calibration> calibration) noexcept {
  calibration_ = std::move(calibration);
  return *this;
}

FastDEM& FastDEM::setOdometryProvider(std::shared_ptr<Odometry> odometry) noexcept {
  odometry_ = std::move(odometry);
  return *this;
}

bool FastDEM::hasTransformProvider() const noexcept {
  return calibration_ != nullptr && odometry_ != nullptr;
}

bool FastDEM::integrate(std::shared_ptr<PointCloud> cloud) {
  // Transform provider validation
  if (!calibration_ || !odometry_) {
    spdlog::error(
        "[FastDEM] Transform providers not set. "
        "Call setTransformProvider() or "
        "setCalibrationProvider()/setOdometryProvider() first, or "
        "use integrate(cloud, T_base_sensor, T_world_base) for explicit "
        "transforms.");
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
  if (cloud.empty()) {
    spdlog::warn("[FastDEM] Received empty cloud. Skipping...");
    return false;
  }

  return integrateImpl(cloud, T_base_sensor, T_world_base);
}

bool FastDEM::integrateImpl(const PointCloud& cloud,
                            const Eigen::Isometry3d& T_base_sensor,
                            const Eigen::Isometry3d& T_world_base) {
  // 1. Preprocess scan (covariance + transform + filter)
  PointCloud points = preprocessScan(cloud, T_base_sensor, T_world_base);
  if (points.empty()) return false;
  if (on_preprocessed_) {
    on_preprocessed_(points);
  }

  // 2. Map update (rasterize + estimate)
  const Eigen::Vector2d robot_position = T_world_base.translation().head<2>();
  auto obs = mapping_->update(points, robot_position);

  // 2.1 Build rasterized scan (one point per cell, z = min_z)
  if (on_rasterized_ && !obs.empty()) {
    on_rasterized_(toPointCloud(obs));
  }

  // 3. (Optional) Raycasting - dynamic object removal
  if (cfg_.raycasting.enabled) {
    const Eigen::Vector3f sensor_origin =
        (T_world_base * T_base_sensor).translation().cast<float>();
    auto ray_scan = nanopcl::filters::voxelGrid(
        points, map_.getResolution(), nanopcl::filters::VoxelMode::ANY);
    applyRaycasting(map_, ray_scan, sensor_origin, cfg_.raycasting);
  }

  return true;
}

PointCloud FastDEM::preprocessScan(const PointCloud& cloud,
                                   const Eigen::Isometry3d& T_base_sensor,
                                   const Eigen::Isometry3d& T_world_base) {
  const auto& z_min = cfg_.point_filter.z_min;
  const auto& z_max = cfg_.point_filter.z_max;
  const auto& range_min = cfg_.point_filter.range_min;
  const auto& range_max = cfg_.point_filter.range_max;

  // Compute covariances in sensor frame, then transform + filter
  PointCloud points = sensor_model_->computeCovariances(cloud);
  points = nanopcl::transformCloud(std::move(points), T_base_sensor);
  points = nanopcl::filters::cropRange(std::move(points), range_min, range_max);
  points = nanopcl::filters::cropZ(std::move(points), z_min, z_max);

  // Transform to map frame
  points = nanopcl::transformCloud(std::move(points), T_world_base,
                                   map_.getFrameId());
  // Rotate covariances to map frame (R·Σ·Rᵀ)
  const Eigen::Matrix3f R =
      (T_world_base * T_base_sensor).rotation().cast<float>();
  for (size_t i : points.indices()) {
    auto& cov = points.covariance(i);
    cov = R * cov * R.transpose();
  }

  return points;
}

void FastDEM::onPreprocessed(CloudCallback callback) {
  on_preprocessed_ = std::move(callback);
}

void FastDEM::onRasterized(CloudCallback callback) {
  on_rasterized_ = std::move(callback);
}

PointCloud FastDEM::toPointCloud(
    const ElevationMapping::CellObservations& observations) const {
  PointCloud cloud;
  cloud.resize(observations.size());
  cloud.setFrameId(map_.getFrameId());

  size_t i = 0;
  for (const auto& [index, cell] : observations) {
    grid_map::Position pos;
    map_.getPosition(index, pos);
    cloud.point(i) = Eigen::Vector3f(pos.x(), pos.y(), cell.min_z);
    ++i;
  }
  return cloud;
}

}  // namespace fastdem
