// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * mapping.cpp
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/mapping/elevation_mapping.hpp"

#include <cmath>
#include <grid_map_core/GridMapMath.hpp>
#include <limits>

namespace fastdem {

ElevationMapping::ElevationMapping(ElevationMap& map, const Config& cfg)
    : map_(map), cfg_(cfg) {
  switch (cfg.estimation_type) {
    case EstimationType::Welford:
      height_estimator_ = MeanEstimation{cfg.sigma_scale};
      break;
    case EstimationType::P2Quantile: {
      const auto& p = cfg.p2;
      height_estimator_ = QuantileEstimation{p.dn0,
                                             p.dn1,
                                             p.dn2,
                                             p.dn3,
                                             p.dn4,
                                             p.elevation_marker,
                                             p.max_sample_count};
      break;
    }
    default: {
      const auto& kf = cfg.kalman;
      height_estimator_ = KalmanEstimation{kf.min_variance, kf.max_variance,
                                           kf.process_noise, cfg.sigma_scale};
      break;
    }
  }
}

void ElevationMapping::update(const PointCloud& rasterized,
                              const Eigen::Vector2d& robot_position) {
  if (rasterized.empty()) return;

  // Move map in LOCAL mode
  if (cfg_.mode == MappingMode::LOCAL) {
    map_.move(grid_map::Position(robot_position.x(), robot_position.y()));
  }

  const bool has_covariance = rasterized.hasCovariance();
  const bool has_intensity = rasterized.hasIntensity();
  const bool has_color = rasterized.hasColor();

  // Ensure intensity/color layers exist and get local pointers
  grid_map::Matrix* intensity_mat = nullptr;
  grid_map::Matrix* color_mat = nullptr;
  if (has_intensity) {
    if (!map_.exists(layer::intensity)) map_.add(layer::intensity, NAN);
    intensity_mat = &map_.get(layer::intensity);
  }
  if (has_color) {
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    if (!map_.exists(layer::color)) map_.add(layer::color, kNaN);
    color_mat = &map_.get(layer::color);
  }

  std::visit(
      [&](auto& estimator) {
        estimator.initialize(map_);

        for (size_t i : rasterized.indices()) {
          auto pt = rasterized.point(i);
          grid_map::Index index;
          if (!map_.getIndex(grid_map::Position(pt.x(), pt.y()), index)) {
            continue;  // Outside map bounds
          }

          // Covariance is already in map frame, extract height variance
          float variance = 0.0f;
          if (has_covariance) {
            variance = rasterized.covariance(i)(2, 2);
          }

          // Height estimation
          estimator.update(index, pt.z(), variance);

          // Intensity: max operation
          if (has_intensity) {
            const int r = index(0);
            const int c = index(1);
            float& stored = (*intensity_mat)(r, c);
            float val = rasterized.intensity(i);
            if (std::isnan(stored) || val > stored) stored = val;
          }

          // Color: overwrite with packed RGB
          if (has_color) {
            const int r = index(0);
            const int c = index(1);
            auto color = rasterized.color(i);
            Eigen::Vector3i rgb(color.r, color.g, color.b);
            float packed;
            grid_map::colorVectorToValue(rgb, packed);
            (*color_mat)(r, c) = packed;
          }
        }

        estimator.finalize();
      },
      height_estimator_);
}

}  // namespace fastdem
