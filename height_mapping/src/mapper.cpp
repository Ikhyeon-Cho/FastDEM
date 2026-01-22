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

#include "height_mapping/api/factory.h"
#include "height_mapping/ops/inpainting.h"
#include "height_mapping/ops/raycasting.h"
#include "height_mapping/ops/spatial_fusion.h"

namespace height_mapping {

Mapper::Mapper(const Config& config) : config_(config) {
  map_ = createHeightMap(config_.map);
  sensor_model_ = createUncertaintyModel(config_.sensor_model);

  // Initialize elevation updater based on config
  const auto& type = config_.estimation.type;
  const float alpha = config_.estimation.alpha;

  if (type == "welford") {
    elevation_updater_ = createWelfordUpdater(alpha);
  } else if (type == "p2_quantile") {
    elevation_updater_ = createQuantileUpdater(config_.estimation.p2);
  } else {
    // Default: kalman_filter (1D Kalman filter with sensor model uncertainty)
    elevation_updater_ = createKalmanUpdater(config_.estimation.kalman, alpha);
  }

  spdlog::debug(
      "[Mapper] Initialized with {}x{}m map, {}m resolution, {} updater",
      config_.map.width, config_.map.height, config_.map.resolution, type);
}

Mapper::~Mapper() = default;

void Mapper::integrate(const PointCloud& cloud_in_sensor,
                       const Transformf& T_base_sensor,
                       const Transformf& T_map_base) {
  // This API is intended to be used as a single function.
  auto data = processScan(cloud_in_sensor, T_base_sensor, T_map_base);
  if (data.points.empty()) return;
  updateMap(data);
}

Mapper::ProcessedData Mapper::processScan(const PointCloud& cloud_in_sensor,  //
                                          const Transformf& T_base_sensor,    //
                                          const Transformf& T_map_base) {
  ProcessedData data;

  if (cloud_in_sensor.empty()) return data;

  // Transform to base frame, then apply spatial filters (meaningful in base)
  auto points = npcl::transformCloud(cloud_in_sensor, T_base_sensor);
  points = npcl::filters::cropZaxis(std::move(points),             //
                                    config_.spatial_filter.z_min,  //
                                    config_.spatial_filter.z_max);
  points = npcl::filters::cropRange(std::move(points),
                                    config_.spatial_filter.range_min,
                                    config_.spatial_filter.range_max);
  // Transform to map frame, then extract single point per cell (max Z)
  points = npcl::transformCloud(std::move(points), T_map_base);
  points = npcl::filters::gridMaxZ(std::move(points), config_.map.resolution);

  data.points = std::move(points);
  data.sensor_origin = (T_map_base * T_base_sensor).translation();
  data.robot_position = T_map_base.translation().head<2>();

  // Compute point uncertainties (in map frame for correct world-Z uncertainty)
  auto uncertainties =
      sensor_model_->computeUncertainties(data.points - data.sensor_origin);
  data.variances = UncertaintyModel::toVariances(uncertainties);

  return data;
}

// =============================================================================
// Thread-safe Public API
// =============================================================================

void Mapper::updateMap(const ProcessedData& data) {
  if (data.points.empty()) return;

  std::lock_guard<std::shared_mutex> lock(map_mutex_);

  // Move map center to robot (local mode only)
  if (config_.map.mode == config::MappingMode::LOCAL) {
    map_->move(
        grid_map::Position(data.robot_position.x(), data.robot_position.y()));
  }

  // Update height map layers
  updateLayers(data.points, data.variances);

  // Post-processing
  ops::applyRaycasting(*map_, data.points, data.sensor_origin,
                       config_.raycasting);
  ops::applyInpainting(*map_, config_.inpainting);
  ops::applySpatialFusion(*map_, config_.spatial_fusion);
}

void Mapper::moveMapOrigin(const Eigen::Vector2f& position) {
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

// =============================================================================
// Private Functions
// =============================================================================

void Mapper::updateLayers(const PointCloud& points,
                          const std::vector<float>& variances) {
  if (points.empty()) return;

  const bool has_variances = !variances.empty();

  std::visit(
      [&](auto& elev_updater) {
        // 1. Initialize
        elev_updater.initialize(*map_);
        intensity_updater_.initialize(*map_);

        // 2. Update each point
        for (const auto& point : points) {
          grid_map::Index index;
          if (!map_->getIndex(grid_map::Position(point.x(), point.y()),
                              index)) {
            continue;
          }
          const float var = has_variances ? variances[point.index()] : 0.0f;
          elev_updater.update(index, point.z(), var);
          if (point.hasIntensity()) {
            intensity_updater_.update(index, point.intensity());
          }
        }

        // 3. Finalize (includes denoising elevation_max internally)
        elev_updater.finalize();
      },
      elevation_updater_);
}

}  // namespace height_mapping
