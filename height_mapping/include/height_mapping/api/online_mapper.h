/*
 * online_mapper.h
 *
 * Provider-based height mapping engine with real-time transform lookup.
 * For standalone use without providers, see Mapper.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ONLINE_MAPPER_H
#define HEIGHT_MAPPING_ONLINE_MAPPER_H

#include <memory>
#include <shared_mutex>

#include "height_mapping/api/config.h"
#include "height_mapping/core.h"

namespace height_mapping {

// Forward declarations
class IExtrinsicsProvider;
class IRobotPoseProvider;

namespace estimator {
class Base;
}

/**
 * @brief Provider-based height mapper with real-time transform lookup.
 *
 * Uses IExtrinsicsProvider and IRobotPoseProvider for automatic transform
 * retrieval. Ideal for ROS integration where TF2 provides transforms.
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
   * @brief Get read-only access to the height map.
   */
  const HeightMap& map() const;

  /**
   * @brief Reset the mapper (clear the map).
   */
  void reset();

 private:
  void initializeHeightEstimator();

  Config config_;
  std::shared_ptr<IExtrinsicsProvider> extrinsics_;
  std::shared_ptr<IRobotPoseProvider> pose_;
  std::unique_ptr<HeightMap> map_;
  std::unique_ptr<estimator::Base> height_estimator_;

  mutable std::shared_mutex map_mutex_;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_ONLINE_MAPPER_H
