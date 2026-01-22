/*
 * mapper.h
 *
 * Height mapper with direct transform input.
 * For real-time transform lookup via providers, see OnlineMapper.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_MAPPER_H
#define HEIGHT_MAPPING_MAPPER_H

#include <memory>
#include <shared_mutex>
#include <variant>

#include "height_mapping/api/config.h"
#include "height_mapping/core.h"
#include "height_mapping/core/updater/elevation_kalman.h"
#include "height_mapping/core/updater/elevation_quantile.h"
#include "height_mapping/core/updater/elevation_welford.h"
#include "height_mapping/core/updater/intensity.h"
#include "height_mapping/sensors/uncertainty_model.h"

namespace height_mapping {

/**
 * @brief Height mapper with direct transform input.
 *
 * Accepts transforms directly in the integrate() call. This simplifies
 * usage when transforms are already available.
 *
 * For real-time transform lookup via providers (e.g., ROS TF2),
 * use OnlineMapper instead.
 *
 * Example usage:
 *   Mapper mapper(config);
 *   mapper.integrate(cloud, T_base_sensor, T_map_base);
 *   const auto& map = mapper.map();
 *
 * For finer control (e.g., callbacks between preprocess and update):
 *   auto data = mapper.processScan(cloud, T_base_sensor, T_map_base);
 *   // ... do something with data.points ...
 *   mapper.updateMap(data);
 */
class Mapper {
 public:
  using Config = MappingConfig;

  /**
   * @brief Result of preprocessing step, ready for map update.
   *
   * Contains all information needed for updateMap():
   * - Filtered points in map frame
   * - Per-point measurement variances
   * - Sensor origin for raycasting
   * - Robot position for local map centering
   */
  struct ProcessedData {
    PointCloud points;              // Filtered points in map frame
    std::vector<float> variances;   // Per-point measurement variances
    Eigen::Vector3f sensor_origin;  // Sensor position in map frame
    Eigen::Vector2f robot_position; // Robot XY position for local mode
  };

  explicit Mapper(const Config& config);
  ~Mapper();

  // Non-copyable
  Mapper(const Mapper&) = delete;
  Mapper& operator=(const Mapper&) = delete;

  /**
   * @brief Integrate a point cloud into the height map.
   *
   * Convenience method that calls processScan() followed by updateMap().
   * For finer control (e.g., callbacks), use these methods separately.
   *
   * @param cloud_in_sensor Point cloud in sensor frame
   * @param T_base_sensor Transform from sensor to base frame
   * @param T_map_base Transform from base to map frame (robot pose)
   */
  void integrate(const PointCloud& cloud_in_sensor,
                 const Transformf& T_base_sensor,
                 const Transformf& T_map_base);

  /**
   * @brief Preprocess point cloud for map integration (lock-free).
   *
   * Performs spatial filtering, coordinate transforms, grid downsampling,
   * and uncertainty computation. This method does NOT acquire any locks
   * and can be called concurrently.
   *
   * @param cloud_in_sensor Point cloud in sensor frame
   * @param T_base_sensor Transform from sensor to base frame
   * @param T_map_base Transform from base to map frame (robot pose)
   * @return ProcessedData ready for updateMap(), or empty if input was empty
   */
  ProcessedData processScan(const PointCloud& cloud_in_sensor,
                                  const Transformf& T_base_sensor,
                                  const Transformf& T_map_base);

  /**
   * @brief Update map with preprocessed data (thread-safe).
   *
   * Acquires exclusive lock and updates map layers, then applies
   * post-processing (raycasting, inpainting, spatial fusion).
   *
   * In local mode, automatically centers the map on robot_position.
   *
   * @param data Preprocessed data from processScan()
   */
  void updateMap(const ProcessedData& data);

  /**
   * @brief Move the map origin to a new position (thread-safe).
   *
   * Acquires exclusive lock internally.
   *
   * @param position New origin position in map frame
   */
  void moveMapOrigin(const Eigen::Vector2f& position);

  /**
   * @brief Get read-only access to the height map (thread-safe).
   */
  const HeightMap& map() const;

  /**
   * @brief Get read-write access to the height map (thread-safe).
   */
  HeightMap& map();

  /**
   * @brief Reset the mapper, clearing all map data (thread-safe).
   */
  void reset();

 private:
  using ElevationUpdater = std::variant<updater::WelfordElevation,
                                        updater::KalmanElevation,
                                        updater::QuantileElevation>;

  void updateLayers(const PointCloud& points,
                    const std::vector<float>& variances);

  Config config_;
  std::unique_ptr<HeightMap> map_;
  std::unique_ptr<UncertaintyModel> sensor_model_;
  ElevationUpdater elevation_updater_;
  updater::Intensity intensity_updater_;

  mutable std::shared_mutex map_mutex_;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_MAPPER_H
