// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * fastdem.hpp
 *
 * FastDEM: Scan-sequential elevation mapping API.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_FASTDEM_HPP
#define FASTDEM_FASTDEM_HPP

#include <functional>
#include <memory>

// Configs
#include "fastdem/config/fastdem.hpp"

// Data types
#include "fastdem/elevation_map.hpp"
#include "fastdem/point_types.hpp"

// Core objects
#include "fastdem/mapping/elevation_mapping.hpp"
#include "fastdem/sensors/sensor_model.hpp"
#include "fastdem/transform_interface.hpp"

namespace fastdem {

/**
 * @brief Scan-sequential elevation mapping API.
 *
 * ## Pipeline (integrate)
 *
 *   SensorModel → preprocessScan → onPreprocessed
 *       → ElevationMapping (rasterize + estimate) → Raycasting
 *
 * ## Callbacks
 *
 * Optional callback can be registered via onPreprocessed()
 * to observe intermediate results (e.g., for visualization or debugging).
 *
 * ## Thread safety
 *
 * This class is **not thread-safe**. The caller is responsible for
 * synchronization when accessing from multiple threads (e.g., wrapping
 * integrate() and map reads in a shared mutex).
 */
class FastDEM {
 public:
  using CloudCallback = std::function<void(const PointCloud&)>;

  /// Construct with default config (use setters to customize)
  explicit FastDEM(ElevationMap& map);

  /// Construct with explicit config
  FastDEM(ElevationMap& map, const Config& cfg);

  ~FastDEM();

  // Non-copyable
  FastDEM(const FastDEM&) = delete;
  FastDEM& operator=(const FastDEM&) = delete;

  /// Set mapping mode (LOCAL: robot-centric, GLOBAL: fixed origin)
  FastDEM& setMappingMode(MappingMode mode);

  /// Set elevation estimator type
  FastDEM& setEstimatorType(EstimationType type);

  /// Set sensor model type
  FastDEM& setSensorModel(SensorType type);

  /// Set custom sensor model (user-defined subclass)
  FastDEM& setSensorModel(std::unique_ptr<SensorModel> model);

  /// Set height filter range in base frame [meters]
  FastDEM& setHeightFilter(float z_min, float z_max) noexcept;

  /// Set range filter (min/max distance from sensor) [meters]
  FastDEM& setRangeFilter(float range_min, float range_max) noexcept;

  /// Enable/disable raycasting (ghost obstacle removal)
  FastDEM& enableRaycasting(bool enabled = true) noexcept;

  /// Set calibration provider (sensor → base static transform)
  FastDEM& setCalibrationProvider(std::shared_ptr<Calibration> calibration);

  /// Set odometry provider (base → world dynamic transform)
  FastDEM& setOdometryProvider(std::shared_ptr<Odometry> odometry);

  /// Set transform provider that implements both interfaces (e.g., ROS TF)
  template <typename T>
  FastDEM& setTransformProvider(std::shared_ptr<T> system) {
    setCalibrationProvider(system);
    setOdometryProvider(system);
    return *this;
  }

  /// Check if transform providers are set (both required)
  bool hasTransformProvider() const noexcept;

  /// Integrate with transform providers (automatic lookup)
  bool integrate(std::shared_ptr<PointCloud> cloud);

  /// Integrate with explicit transforms
  bool integrate(const PointCloud& cloud,
                 const Eigen::Isometry3d& T_base_sensor,
                 const Eigen::Isometry3d& T_world_base);

  /// Register callback: fired after preprocessing (covariance + transform +
  /// filter), before map update
  void onPreprocessed(CloudCallback callback);

  /// Register callback: fired after rasterization (one point per cell,
  /// z = min height), before raycasting
  void onRasterized(CloudCallback callback);

 private:
  /// Transform, filter, and rotate covariances to map frame
  PointCloud preprocessScan(const PointCloud& cloud,
                            const Eigen::Isometry3d& T_base_sensor,
                            const Eigen::Isometry3d& T_world_base);

  /// Core integration logic (assumes valid inputs)
  bool integrateImpl(const PointCloud& cloud,
                     const Eigen::Isometry3d& T_base_sensor,
                     const Eigen::Isometry3d& T_world_base);

  /// Convert rasterized cell observations to point cloud (one point per cell)
  PointCloud toPointCloud(const ElevationMapping::CellObservations& obs) const;

  ElevationMap& map_;
  Config cfg_;

  // Core components
  std::unique_ptr<fastdem::SensorModel> sensor_model_;
  std::unique_ptr<fastdem::ElevationMapping> mapping_;

  // Transform providers (null when using explicit transforms)
  std::shared_ptr<Calibration> calibration_;
  std::shared_ptr<Odometry> odometry_;

  // Optional callbacks
  CloudCallback on_preprocessed_;
  CloudCallback on_rasterized_;
};

}  // namespace fastdem

#endif  // FASTDEM_FASTDEM_HPP
