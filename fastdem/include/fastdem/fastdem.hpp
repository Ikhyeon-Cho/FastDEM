// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * fastdem.hpp
 *
 * FastDEM: Unified height mapping API for online and offline use.
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
#include <shared_mutex>

// Configs
#include "fastdem/config/fastdem.hpp"

// Data types
#include "fastdem/elevation_map.hpp"
#include "fastdem/point_types.hpp"

// Core objects
#include "fastdem/mapping/elevation_mapping.hpp"
#include "fastdem/mapping/rasterization.hpp"
#include "fastdem/sensors/sensor_model.hpp"
#include "fastdem/transform_interface.hpp"

namespace fastdem {

/**
 * @brief Unified elevation mapping API for online and offline use.
 *
 * ## Thread safety
 *
 * **Thread-safe** (guarded by internal shared_mutex):
 * - integrate() — both online and offline overloads (exclusive lock)
 * - withMap() — read-only access to the map (shared lock)
 * - setCalibrationSystem(), setOdometrySystem(), setTransformSystem()
 * - hasTransformSystems()
 *
 * **Not thread-safe** — call these before integration or from a single thread:
 * - Configuration setters: setMappingMode(), setEstimatorType(),
 *   setSensorModel(), setHeightRange(), setDistanceRange(),
 *   enableRaycasting(), enableInpainting(), enableUncertaintyFusion()
 * - setProcessedCloudCallback()
 *
 * **Typical usage pattern**: configure once from the main thread, then call
 * integrate() from a sensor callback thread. Use withMap() for thread-safe
 * read access (e.g., publishing). Changing transform providers at runtime
 * is safe.
 */
class FastDEM {
 public:
  using Config = MappingConfig;
  using ProcessedCloudCallback = std::function<void(const PointCloud&)>;

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

  /// Set height filter range in base frame [meters]
  FastDEM& setHeightRange(float z_min, float z_max) noexcept;

  /// Set distance filter range from sensor [meters]
  FastDEM& setDistanceRange(float range_min, float range_max) noexcept;

  /// Enable/disable raycasting (ghost obstacle removal)
  FastDEM& enableRaycasting(bool enabled = true) noexcept;

  /// Enable/disable inpainting (hole filling)
  FastDEM& enableInpainting(bool enabled = true) noexcept;

  /// Enable/disable spatial fusion (uncertainty smoothing)
  FastDEM& enableUncertaintyFusion(bool enabled = true) noexcept;

  /// Set calibration system (sensor → base static transform)
  FastDEM& setCalibrationSystem(std::shared_ptr<Calibration> calibration);

  /// Set odometry system (base → world dynamic transform)
  FastDEM& setOdometrySystem(std::shared_ptr<Odometry> odometry);

  /// Set transform system that implements both interfaces (e.g., ROS TF)
  template <typename T>
  FastDEM& setTransformSystem(std::shared_ptr<T> system) {
    setCalibrationSystem(system);
    setOdometrySystem(system);
    return *this;
  }

  /// Check if online mode is ready (both systems set)
  bool hasTransformSystems() const noexcept;

  /// Online mode: integrate with automatic transform lookup
  bool integrate(std::shared_ptr<PointCloud> cloud);

  /// Offline mode: integrate with explicit transforms
  bool integrate(const PointCloud& cloud,
                 const Eigen::Isometry3d& T_base_sensor,
                 const Eigen::Isometry3d& T_world_base);

  /// Set callback for processed cloud (after preprocess, before map update)
  void setProcessedCloudCallback(ProcessedCloudCallback callback);

  /// Thread-safe read access to the elevation map.
  /// Acquires a shared lock, so multiple readers can run concurrently
  /// while integrate() is blocked.
  template <typename F>
  decltype(auto) withMap(F&& fn) const {
    std::shared_lock lock(map_mutex_);
    return fn(map_);
  }

 private:
  /// Core integration logic (assumes valid inputs)
  bool integrateImpl(const PointCloud& cloud,
                     const Eigen::Isometry3d& T_base_sensor,
                     const Eigen::Isometry3d& T_world_base);

  ElevationMap& map_;
  Config cfg_;

  // Core components
  std::unique_ptr<fastdem::SensorModel> sensor_model_;
  std::unique_ptr<fastdem::Rasterization> rasterization_;
  std::unique_ptr<fastdem::ElevationMapping> mapping_;

  // Online Transform systems (null in offline mode)
  std::shared_ptr<Calibration> calibration_;
  std::shared_ptr<Odometry> odometry_;

  // Optional callback
  ProcessedCloudCallback processed_cloud_callback_;

  // Thread safety
  mutable std::shared_mutex map_mutex_;
};

// ─── Free functions: direct rasterization ────────────────────────────

/**
 * @brief Rasterize world-frame point cloud directly into elevation map.
 *
 * Bypasses the full pipeline (no sensor model, transform, or estimation).
 * Simply bins points into grid cells and writes representative z values.
 *
 * @param cloud Point cloud in map/world frame
 * @param map Pre-sized ElevationMap (must have geometry set)
 * @param method Rasterization method (Max, Min, or Mean)
 */
void rasterize(const PointCloud& cloud, ElevationMap& map,
               RasterMethod method = RasterMethod::Max);

/**
 * @brief Rasterize world-frame point cloud into an auto-sized elevation map.
 *
 * Map geometry is determined by the cloud's XY bounding box.
 *
 * @param cloud Point cloud in map/world frame
 * @param resolution Grid cell size in meters
 * @param method Rasterization method (Max, Min, or Mean)
 * @return ElevationMap sized to fit the cloud
 */
ElevationMap rasterize(const PointCloud& cloud, float resolution,
                       RasterMethod method = RasterMethod::Max);

}  // namespace fastdem

#endif  // FASTDEM_FASTDEM_HPP
