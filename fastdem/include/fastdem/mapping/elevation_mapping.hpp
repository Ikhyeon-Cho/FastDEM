// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * elevation_mapping.hpp
 *
 * Elevation mapping: updates all map layers from rasterized point cloud.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_MAPPING_ELEVATION_MAPPING_HPP
#define FASTDEM_MAPPING_ELEVATION_MAPPING_HPP

#include <memory>
#include <variant>

#include "fastdem/config/mapping.hpp"
#include "fastdem/elevation_map.hpp"
#include "fastdem/mapping/kalman_estimation.hpp"
#include "fastdem/mapping/mean_estimation.hpp"
#include "fastdem/mapping/quantile_estimation.hpp"
#include "fastdem/point_types.hpp"

namespace fastdem {

/**
 * @brief Updates all ElevationMap layers from rasterized points.
 *
 * Handles map movement (LOCAL mode), cell index calculation, and layer updates
 * for elevation (via HeightEstimator variant), intensity, and color.
 *
 * @code
 *   auto mapping = ElevationMapping::create(map, config);
 *   auto rasterized = rasterization.process(cloud, map);
 *   mapping->update(rasterized, robot_position);
 * @endcode
 */
class ElevationMapping {
 public:
  using Config = config::Mapping;
  using HeightEstimator =
      std::variant<MeanEstimation, KalmanEstimation, QuantileEstimation>;

  ElevationMapping(ElevationMap& map, const Config& cfg);

  /**
   * @brief Update map layers from rasterized point cloud.
   *
   * In LOCAL mode, moves map center to robot_position before updating.
   * In GLOBAL mode, robot_position is ignored.
   *
   * @param rasterized Points from Rasterization::process()
   * @param robot_position Robot position in world frame (used in LOCAL mode)
   */
  void update(const PointCloud& rasterized,
              const Eigen::Vector2d& robot_position);

 private:
  ElevationMap& map_;
  Config cfg_;
  HeightEstimator height_estimator_;
};

/// Factory function for consistent creation pattern
inline std::unique_ptr<ElevationMapping> createElevationMapping(
    ElevationMap& map, const config::Mapping& cfg) {
  return std::make_unique<ElevationMapping>(map, cfg);
}

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_ELEVATION_MAPPING_HPP
