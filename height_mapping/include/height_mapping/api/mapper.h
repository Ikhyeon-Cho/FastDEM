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

#include "height_mapping/api/config.h"
#include "height_mapping/core.h"

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
 */
class Mapper {
 public:
  using Config = MappingConfig;

  explicit Mapper(const Config& config);
  ~Mapper();

  // Non-copyable
  Mapper(const Mapper&) = delete;
  Mapper& operator=(const Mapper&) = delete;

  /**
   * @brief Integrate a point cloud into the height map.
   *
   * @param cloud_in_sensor Point cloud in sensor frame
   * @param T_base_sensor Transform from sensor to base frame
   * @param T_map_base Transform from base to map frame (robot pose)
   */
  void integrate(const PointCloud& cloud_in_sensor,
                 const Transformf& T_base_sensor, const Transformf& T_map_base);

  /**
   * @brief Move the map origin to a new position (robot-centric mode).
   *
   * @param position New origin position in map frame
   */
  void moveOrigin(const Eigen::Vector2f& position);

  /**
   * @brief Get read-only access to the height map.
   */
  const HeightMap& map() const;

  /**
   * @brief Get read-write access to the height map.
   */
  HeightMap& map();

  /**
   * @brief Reset the mapper (clear the map).
   */
  void reset();

 private:
  void initializeHeightEstimator();

  Config config_;
  std::unique_ptr<HeightMap> map_;
  std::unique_ptr<estimator::Base> height_estimator_;

  mutable std::shared_mutex map_mutex_;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_MAPPER_H
