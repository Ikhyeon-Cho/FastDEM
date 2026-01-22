/*
 * online_mapper.h
 *
 * Provider-based height mapping engine with real-time transform lookup.
 * Internally delegates to Mapper for actual processing.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ONLINE_MAPPER_H
#define HEIGHT_MAPPING_ONLINE_MAPPER_H

#include <functional>
#include <memory>

#include "height_mapping/api/config.h"
#include "height_mapping/api/mapper.h"

namespace height_mapping {

// Forward declarations
class IExtrinsicsProvider;
class IRobotPoseProvider;

/**
 * @brief Provider-based height mapper with real-time transform lookup.
 *
 * Uses IExtrinsicsProvider and IRobotPoseProvider for automatic transform
 * retrieval. Ideal for ROS integration where TF2 provides transforms.
 *
 * Internally uses Mapper for all processing logic (Composition pattern).
 * OnlineMapper's responsibility is solely transform resolution.
 *
 * For direct transform input, use Mapper instead.
 *
 * Example usage:
 *   auto mapper = std::make_unique<OnlineMapper>(config, extrinsics, pose);
 *   mapper->integrate(cloud);
 *   const auto& map = mapper->map();
 */
class OnlineMapper {
 public:
  using Config = MappingConfig;
  using ProcessedScanCallback = std::function<void(const PointCloud&)>;

  OnlineMapper(const Config& config,
               std::shared_ptr<IExtrinsicsProvider> extrinsics,
               std::shared_ptr<IRobotPoseProvider> pose);
  ~OnlineMapper();

  // Non-copyable
  OnlineMapper(const OnlineMapper&) = delete;
  OnlineMapper& operator=(const OnlineMapper&) = delete;

  /**
   * @brief Integrate a point cloud into the height map.
   *
   * Transforms are automatically retrieved from the providers.
   *
   * @param cloud Point cloud with timestamp (frame_id used for extrinsics)
   */
  void integrate(std::shared_ptr<PointCloud> cloud);

  /**
   * @brief Get read-only access to the height map (thread-safe).
   */
  const HeightMap& map() const;

  /**
   * @brief Reset the mapper, clearing all map data (thread-safe).
   */
  void reset();

  /**
   * @brief Move the map origin to a new position (thread-safe).
   */
  void moveMapOrigin(const Eigen::Vector2f& position);

  /**
   * @brief Set callback to receive processed scan.
   *
   * The callback is invoked after processScan() (filtering, transforms,
   * gridMaxZ) but before updateMap().
   *
   * @param callback Function to call with processed point cloud
   */
  void setProcessedScanCallback(ProcessedScanCallback callback);

 private:
  std::shared_ptr<IExtrinsicsProvider> extrinsics_;
  std::shared_ptr<IRobotPoseProvider> pose_;

  // Core mapper (Composition - delegates all processing)
  std::unique_ptr<Mapper> mapper_;

  ProcessedScanCallback processed_scan_callback_;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_ONLINE_MAPPER_H
